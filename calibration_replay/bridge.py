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

_PACKAGE_NAME = "_calibration_replay_h2_backend"


def _load_backend(project: str | Path):
    root = Path(project).expanduser().resolve()
    init_file = root / "backend" / "__init__.py"
    if not init_file.is_file():
        raise FileNotFoundError(f"hand_eye_3D backend not found under {root}")
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

    def engage(self, arm: str | None = None) -> None:
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
        }


class H2ArmBridge:
    """Isolated adapter; this service is the only rt/arm_sdk publisher."""

    source = "h2"

    def __init__(self, project: str | Path, network_interface: str | None, arm: str = "right"):
        self.project = str(Path(project).expanduser().resolve())
        self.network_interface = network_interface
        self.arm = arm
        self.joint_names = joint_names_for(arm)
        self.limits = read_urdf_limits(self.project, arm)
        self._controller = None
        self._reader = None
        self._lock = threading.RLock()

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

    def engage(self, arm: str | None = None) -> None:
        with self._lock:
            if arm is not None:
                self.select_arm(arm)
            if self._controller is not None:
                return
            arm_module = _module(self.project, "arm")

            class TimestampedH2ArmController(arm_module.H2ArmController):
                def _on_low_state(inner_self, message) -> None:
                    inner_self._calibration_state_at = time.monotonic()
                    inner_self._calibration_gyro = _imu_gyro(message)
                    super()._on_low_state(message)

            controller = TimestampedH2ArmController(
                arm=self.arm,
                network_interface=self.network_interface,
                max_speed_rad_s=0.30,
                hand_move_kd=2.0,
                grav_alpha=1.0,
                payload_kg=0.0,
                grav_in_float=True,
            )
            controller.start()
            self._controller = controller
            self.joint_names = list(controller.joint_names)
            self.limits = controller.limits.tolist()

    def disarm(self) -> None:
        with self._lock:
            controller, self._controller = self._controller, None
        if controller is not None:
            controller.shutdown()

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
            }
        q = self._read_only_q()
        return {
            "q": q,
            "dq": None,
            "timestamp": time.monotonic(),
            "cmd_gap_rad": None,
            "gyro_rad_s": None,
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
            }
        status = controller.status()
        status["source"] = self.source
        status["motion_enabled"] = bool(status.get("jog_enabled"))
        status["guide"] = bool(status.get("float"))
        return status
