from __future__ import annotations

import importlib
import importlib.util
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .models import ARMS, joint_names_for
from .payload import PayloadStore

_PACKAGE_NAME = "_calibration_replay_h2_backend"

# 接管时的 PD 刚度，取 IK_replay reach_server 的默认值（--arm-kp 140 --arm-kd 3.0，腕 50/2.0）
ARM_KP = 140.0
ARM_KD = 3.0
ARM_KP_WRIST = 50.0
ARM_KD_WRIST = 2.0


def _load_backend(project: str | Path):
    root = Path(project).expanduser().resolve()
    init_file = root / "backend" / "__init__.py"
    if not init_file.is_file():
        init_file = root / "calib_workstation" / "calib3d" / "__init__.py"
    if not init_file.is_file():
        raise FileNotFoundError(f"H2 calibration runtime not found under {root}")
    if _PACKAGE_NAME not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            _PACKAGE_NAME,
            init_file,
            submodule_search_locations=[str(init_file.parent)],
        )
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load {init_file}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[_PACKAGE_NAME] = module
        spec.loader.exec_module(module)
    return root


def _module(project: str | Path, name: str):
    _load_backend(project)
    return importlib.import_module(f"{_PACKAGE_NAME}.{name}")


def read_urdf_limits(project: str | Path, arm: str = "right") -> list[list[float]] | None:
    joint_names = joint_names_for(arm)
    path = Path(project).expanduser().resolve() / "assets/robots/h2/robot.urdf"
    if not path.is_file():
        return None
    by_name: dict[str, list[float]] = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            by_name[joint.get("name", "")] = [
                float(limit.get("lower")),
                float(limit.get("upper")),
            ]
    return [by_name[name] for name in joint_names] if all(name in by_name for name in joint_names) else None


def _imu_gyro(low_state) -> list[float] | None:
    """Torso IMU angular rate from rt/lowstate, None when the message lacks it."""
    imu = getattr(low_state, "imu_state", None)
    gyro = getattr(imu, "gyroscope", None)
    if gyro is None:
        return None
    try:
        values = [float(v) for v in gyro]
    except (TypeError, ValueError):
        return None
    return values if len(values) == 3 else None


class MockArmBridge:
    source = "mock"

    def __init__(self, arm: str = "right"):
        self.arm = arm
        self.joint_names = joint_names_for(arm)
        self.limits = [[-3.0, 3.0] for _ in self.joint_names]
        self._q = [0.0] * 7
        self._engaged = False
        self._guide = False
        self._jog = False
        self._lock = threading.Lock()

    def select_arm(self, arm: str) -> None:
        """Which arm read-only queries refer to; refuses to switch while engaged."""
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
        if self._engaged and arm != self.arm:
            raise RuntimeError(f"{self.arm} arm is engaged; disarm before switching to {arm}")
        self.arm = arm
        self.joint_names = joint_names_for(arm)

    def engage(self, arm: str | None = None, gravity_profile: dict | None = None) -> None:
        if arm is not None:
            self.select_arm(arm)
        self._engaged = True

    def disarm(self) -> None:
        self._engaged = self._guide = self._jog = False

    def guide(self) -> bool:
        if not self._engaged:
            return False
        self._jog = False
        self._guide = True
        return True

    def catch_hold(self) -> None:
        self._guide = self._jog = False

    def enable_motion(self) -> None:
        if not self._engaged:
            raise RuntimeError("arm is not engaged")
        self._guide = False
        self._jog = True

    def set_target(self, q: list[float]) -> bool:
        if not self._jog:
            return False
        with self._lock:
            self._q = [float(v) for v in q]
        return True

    def stop_hold(self) -> None:
        self._guide = self._jog = False

    def read_sample(self) -> dict[str, Any]:
        with self._lock:
            q = list(self._q)
        return {
            "q": q,
            "dq": [0.0] * 7,
            "timestamp": time.monotonic(),
            "cmd_gap_rad": 0.0,
            "gyro_rad_s": [0.0, 0.0, 0.0],
        }

    def payload_info(self) -> dict[str, Any]:
        return {"enabled": False, "active": False, "arm": self.arm, "reason": "mock 无重力前馈"}

    def reload_payload(self) -> dict[str, Any]:
        return self.payload_info()

    def status(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "arm": self.arm,
            "engaged": self._engaged,
            "guide": self._guide,
            "motion_enabled": self._jog,
            "joint_names": self.joint_names,
            "measured_rad": self.read_sample()["q"],
            "limits_rad": self.limits,
            "payload": self.payload_info(),
        }


class H2ArmBridge:
    """Isolated adapter; this service is the only rt/arm_sdk publisher."""

    source = "h2"

    def __init__(self, project: str | Path, network_interface: str | None, arm: str = "right",
                 payload_store: PayloadStore | None = None):
        self.project = str(Path(project).expanduser().resolve())
        self.network_interface = network_interface
        self.arm = arm
        self.joint_names = joint_names_for(arm)
        self.limits = read_urdf_limits(self.project, arm)
        self._controller = None
        self._reader = None
        self._lock = threading.RLock()
        # 末端负载重力补偿（arm_payload_gravity 标定结果）；None = 禁用，沿用作者原前馈
        self.payload_store = payload_store or PayloadStore(None, None)
        self._gravity_profile: dict[str, Any] | None = None
        self._payload: dict[str, Any] = {"enabled": self.payload_store.enabled, "active": False, "arm": arm,
                                         "reason": "未接管"}

    def select_arm(self, arm: str) -> None:
        """Which arm read-only queries (joints, record node) refer to. Both arms
        are first-class: switching only requires that nothing is engaged."""
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
        with self._lock:
            if self._controller is not None and arm != self.arm:
                raise RuntimeError(f"{self.arm} arm is engaged; disarm before switching to {arm}")
            if arm == self.arm:
                return
            self.arm = arm
            self.joint_names = joint_names_for(arm)
            self.limits = read_urdf_limits(self.project, arm)

    def _read_only_q(self) -> list[float]:
        """One rt/lowstate subscription serves both arms: pick the motor block
        for the currently selected arm instead of re-subscribing per arm."""
        with self._lock:
            if self._reader is None:
                robot = _module(self.project, "robot")
                self._reader = robot.H2PoseProvider(
                    network_interface=self.network_interface,
                    arm=self.arm,
                )
            reader = self._reader
            arm = self.arm
        robot = _module(self.project, "robot")
        indices = (
            robot.H2_RIGHT_ARM_MOTOR_INDICES if arm == "right" else robot.H2_LEFT_ARM_MOTOR_INDICES
        )
        with reader._lock:
            state = reader._low_state
        if state is None:
            raise RuntimeError("还没收到 rt/lowstate")
        return [float(state.motor_state[i].q) for i in indices]

    def engage(self, arm: str | None = None, gravity_profile: dict | None = None) -> None:
        with self._lock:
            if arm is not None:
                self.select_arm(arm)
            if self._controller is not None:
                if gravity_profile != self._gravity_profile:
                    raise RuntimeError("机械臂已接管；切换重力补偿前请先解除接管")
                return
            self._gravity_profile = gravity_profile
            arm_module = _module(self.project, "arm")

            class TimestampedH2ArmController(arm_module.H2ArmController):
                def _on_low_state(inner_self, message) -> None:
                    inner_self._calibration_state_at = time.monotonic()
                    inner_self._calibration_gyro = _imu_gyro(message)
                    super()._on_low_state(message)

            profile_parameters = (
                dict(gravity_profile.get("parameters") or {})
                if gravity_profile is not None else None
            )
            gravity_parameters = {
                "grav_alpha": 1.0,
                "payload_kg": 0.0,
                "payload_com_m": None,
                "payload_link": None,
                "excluded_subtree_link": None,
                "grav_in_float": True,
                "use_imu_gravity": False,
            }
            if profile_parameters:
                gravity_parameters.update({
                    key: profile_parameters.get(key)
                    for key in gravity_parameters
                    if key in profile_parameters
                })
            controller = TimestampedH2ArmController(
                arm=self.arm,
                network_interface=self.network_interface,
                max_speed_rad_s=0.30,
                # 与 IK_replay reach 服务一致的刚度（默认 80/1.5 偏软，起停晃动、下垂大）
                kp=ARM_KP,
                kd=ARM_KD,
                kp_wrist=ARM_KP_WRIST,
                kd_wrist=ARM_KD_WRIST,
                hand_move_kd=2.0,
                **gravity_parameters,
            )
            # 负载补偿在 start() 前注入：控制线程第一周期就带负载前馈，接管瞬间不会先下垂再抬起
            if gravity_profile is None:
                self._payload = self.payload_store.apply(controller, self.arm)
            elif gravity_profile.get("mode") == "profile":
                self._payload = {
                    "enabled": True,
                    "active": True,
                    "arm": self.arm,
                    "source": "18000_gravity_profile",
                    "version": gravity_profile.get("version"),
                    "label": gravity_profile.get("label"),
                    "mass_kg": gravity_parameters["payload_kg"],
                    "com_m": gravity_parameters["payload_com_m"],
                    "alpha": gravity_parameters["grav_alpha"],
                    "parameters": gravity_parameters,
                }
            else:
                self._payload = {
                    "enabled": False,
                    "active": False,
                    "arm": self.arm,
                    "source": "task_override",
                    "version": "none",
                    "label": "不加载额外负载补偿",
                    "reason": "本次任务保留机器人基础重力前馈，不加载额外负载补偿",
                    "parameters": gravity_parameters,
                }
            controller.start()
            self._controller = controller
            self.joint_names = list(controller.joint_names)
            self.limits = controller.limits.tolist()

    def disarm(self) -> None:
        with self._lock:
            controller, self._controller = self._controller, None
            self._gravity_profile = None
            self._payload = {"enabled": self.payload_store.enabled, "active": False, "arm": self.arm, "reason": "未接管"}
        if controller is not None:
            controller.shutdown()

    def payload_info(self) -> dict[str, Any]:
        with self._lock:
            info = dict(self._payload)
            profile_mode = self._gravity_profile is not None
        if not profile_mode:
            info["file"] = self.payload_store.describe_file(self.arm)
        return info

    def reload_payload(self) -> dict[str, Any]:
        """重新读 payload_<arm>.json 并热替换前馈（10183 里「应用并保存」之后调用，不必解除接管）。"""
        with self._lock:
            controller = self._controller
            if self._gravity_profile is not None:
                return self.payload_info()
            if controller is None:
                self._payload = {"enabled": self.payload_store.enabled, "active": False, "arm": self.arm,
                                 "reason": "未接管；接管时会自动加载"}
            else:
                self._payload = self.payload_store.apply(controller, self.arm)
        return self.payload_info()

    def guide(self) -> bool:
        with self._lock:
            if self._controller is None:
                return False
            self._controller.disable_jog()
            return bool(self._controller.enter_hand_move())

    def catch_hold(self) -> None:
        with self._lock:
            if self._controller is None:
                raise RuntimeError("arm is not engaged")
            self._controller.stop()

    def enable_motion(self) -> None:
        with self._lock:
            if self._controller is None:
                raise RuntimeError("arm is not engaged")
            self._controller.enable_jog()

    def set_target(self, q: list[float]) -> bool:
        with self._lock:
            if self._controller is None:
                return False
            return bool(self._controller.set_target(q))

    def stop_hold(self) -> None:
        with self._lock:
            if self._controller is not None:
                self._controller.stop()

    def read_sample(self) -> dict[str, Any]:
        with self._lock:
            controller = self._controller
        if controller is not None:
            status = controller.status()
            desired = status.get("desired_rad")
            cmd = status.get("cmd_rad")
            cmd_gap = (
                max(abs(float(a) - float(b)) for a, b in zip(desired, cmd))
                if desired is not None and cmd is not None
                else None
            )
            return {
                "q": status["measured_rad"],
                "dq": status.get("measured_dq_rad_s"),
                "timestamp": float(
                    getattr(controller, "_calibration_state_at", 0.0)
                ),
                "cmd_gap_rad": cmd_gap,
                "gyro_rad_s": getattr(controller, "_calibration_gyro", None),
                # 接管期间额外暴露：控制目标、限速后下发值、电机估计力矩（/test 读数页用）
                "desired_rad": desired,
                "cmd_rad": cmd,
                "tau_est_nm": status.get("tau_est_nm"),
                "engaged": True,
            }
        q = self._read_only_q()
        return {
            "q": q,
            "dq": None,
            "timestamp": time.monotonic(),
            "cmd_gap_rad": None,
            "gyro_rad_s": None,
            "desired_rad": None,
            "cmd_rad": None,
            "tau_est_nm": None,
            "engaged": False,
        }

    def status(self) -> dict[str, Any]:
        with self._lock:
            controller = self._controller
        if controller is None:
            sample = self.read_sample()
            return {
                "source": self.source,
                "arm": self.arm,
                "engaged": False,
                "joint_names": self.joint_names,
                "measured_rad": sample["q"],
                "limits_rad": self.limits,
                "payload": self.payload_info(),
            }
        status = controller.status()
        status["source"] = self.source
        status["motion_enabled"] = bool(status.get("jog_enabled"))
        status["guide"] = bool(status.get("float"))
        status["payload"] = self.payload_info()
        return status
