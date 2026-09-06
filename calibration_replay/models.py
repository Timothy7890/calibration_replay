from __future__ import annotations

import math
import re
import uuid
from dataclasses import asdict, dataclass, field
from typing import Any, Literal

ARMS = ("left", "right")
Arm = Literal["left", "right"]
ARM_LABELS = {"left": "左臂", "right": "右臂"}
JOINT_SUFFIXES = [
    "shoulder_pitch_joint",
    "shoulder_roll_joint",
    "shoulder_yaw_joint",
    "elbow_joint",
    "wrist_roll_joint",
    "wrist_pitch_joint",
    "wrist_yaw_joint",
]
# H2 的左右臂关节轴向相同、安装沿 Y 镜像：绕 Y 的 pitch/elbow 不变，绕 X/Z 的
# roll/yaw 取反（限位也正好只有 shoulder_roll 反号）。
MIRROR_SIGNS = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0]


def joint_names_for(arm: str) -> list[str]:
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
    return [f"{arm}_{suffix}" for suffix in JOINT_SUFFIXES]


def other_arm(arm: str) -> str:
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
    return "left" if arm == "right" else "right"


def mirror_q(q: list[float]) -> list[float]:
    return [float(v) * sign for v, sign in zip(q, MIRROR_SIGNS)]


# Backwards-compatible alias: plans saved before the arm field existed are right-arm plans.
JOINT_NAMES = joint_names_for("right")
PLAN_VERSION = 1
PlanTarget = Literal["hand_eye_2D_head", "hand_eye_2D_waist", "hand_eye_3D"]
NodeRole = Literal["home", "transit", "sample"]
SAFE_EXTERNAL_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


@dataclass
class MotionConfig:
    vmax_rad_s: float = 0.15
    amax_rad_s2: float = 0.35
    min_duration_s: float = 1.0
    max_start_delta_rad: float = 0.15
    max_adjacent_delta_rad: float = 1.25
    rate_hz: float = 50.0


@dataclass
class StabilityConfig:
    """Arrival = the arm itself has stopped moving (encoders + torso IMU).

    The planned joint vector is only a reference: the measured pose is what gets
    recorded, and ``max_error_rad`` merely flags a large deviation in the
    certificate/log, it never blocks the capture.
    """

    window_s: float = 0.5               # 自身静止需连续保持的时长
    max_velocity_rad_s: float = 0.08    # 窗口均值编码器速度上限（原始 dq 噪声约 ±0.08）
    max_range_rad: float = 0.015        # 窗口内每个关节的实测漂移上限
    max_gyro_rad_s: float = 0.10        # 躯干 IMU 角速度上限（lowstate 有 IMU 时才判）
    freshness_s: float = 0.15           # 实测数据的最大陈旧时间
    command_settle_s: float = 5.0       # 等待控制器把限速指令送达（desired≈cmd）的最长时间
    timeout_s: float = 15.0             # 总超时
    max_error_rad: float = 0.05         # 仅记录/告警：实测与规划参考的偏差
    capture_delay_s: float = 0.5        # 判定静止后再等这么久才拍摄（可为 0）


@dataclass
class PlanNode:
    id: str
    name: str
    role: NodeRole
    q_rad: list[float]
    enabled: bool = True
    source: str = "manual"
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def create(cls, name: str, role: NodeRole, q_rad: list[float], **kwargs):
        return cls(id=uuid.uuid4().hex[:12], name=name, role=role, q_rad=q_rad, **kwargs)


@dataclass
class Plan:
    id: str
    name: str
    target: PlanTarget
    base_url: str
    arm: Arm = "right"
    camera_serial: str | None = None
    nodes: list[PlanNode] = field(default_factory=list)
    motion: MotionConfig = field(default_factory=MotionConfig)
    stability: StabilityConfig = field(default_factory=StabilityConfig)
    require_corners: bool = True
    draft: bool = True
    version: int = PLAN_VERSION
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def create(cls, name: str, target: PlanTarget, base_url: str, arm: str = "right"):
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
        return cls(id=uuid.uuid4().hex[:12], name=name, target=target, base_url=base_url, arm=arm)

    @property
    def joint_names(self) -> list[str]:
        return joint_names_for(self.arm)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Plan":
        return cls(
            id=str(value["id"]),
            name=str(value["name"]),
            target=value["target"],
            base_url=str(value["base_url"]).rstrip("/"),
            arm=str(value.get("arm") or "right"),
            camera_serial=(
                str(value["camera_serial"]).strip()
                if value.get("camera_serial") not in (None, "")
                else None
            ),
            nodes=[PlanNode(**node) for node in value.get("nodes", [])],
            motion=MotionConfig(**value.get("motion", {})),
            stability=StabilityConfig(**value.get("stability", {})),
            require_corners=bool(value.get("require_corners", True)),
            draft=bool(value.get("draft", True)),
            version=int(value.get("version", PLAN_VERSION)),
            metadata=dict(value.get("metadata", {})),
        )


def _positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and > 0")


def validate_q(q: list[float], label: str = "q_rad") -> None:
    if len(q) != 7:
        raise ValueError(f"{label} must contain exactly 7 joints")
    if not all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in q):
        raise ValueError(f"{label} must contain only finite numbers")


def validate_plan(
    plan: Plan,
    *,
    limits: list[list[float]] | None = None,
    require_home: bool = False,
    require_capture_ready: bool = False,
    check_adjacent: bool = True,
) -> list[str]:
    """Structural checks always run; ``check_adjacent=False`` lets a draft be
    saved with oversized gaps so the operator can add transit nodes afterwards.
    Route building, validation and runs always enforce the adjacent limit."""
    errors: list[str] = []
    if plan.version != PLAN_VERSION:
        errors.append(f"unsupported plan version {plan.version}")
    if plan.target not in ("hand_eye_2D_head", "hand_eye_2D_waist", "hand_eye_3D"):
        errors.append(f"unsupported target {plan.target!r}")
    if plan.arm not in ARMS:
        errors.append(f"unsupported arm {plan.arm!r}, must be left or right")
    if not plan.name.strip():
        errors.append("plan name is required")
    if not plan.base_url.startswith(("http://", "https://")):
        errors.append("base_url must start with http:// or https://")
    if (
        require_capture_ready
        and plan.target.startswith("hand_eye_2D")
        and not plan.camera_serial
    ):
        errors.append("2D plans require an explicit camera_serial before run")

    for key, value in asdict(plan.motion).items():
        try:
            _positive(float(value), f"motion.{key}")
        except ValueError as exc:
            errors.append(str(exc))
    for key, value in asdict(plan.stability).items():
        try:
            if key == "capture_delay_s":
                if not math.isfinite(float(value)) or float(value) < 0:
                    raise ValueError("stability.capture_delay_s must be finite and >= 0")
            else:
                _positive(float(value), f"stability.{key}")
        except ValueError as exc:
            errors.append(str(exc))

    seen: set[str] = set()
    homes: list[PlanNode] = []
    for index, node in enumerate(plan.nodes):
        if not node.id or node.id in seen:
            errors.append(f"node[{index}] has missing or duplicate id {node.id!r}")
        elif not SAFE_EXTERNAL_ID_RE.fullmatch(node.id):
            errors.append(
                f"node[{index}] id must use 1-128 safe characters "
                "(letters, digits, '.', '_' or '-')"
            )
        seen.add(node.id)
        if node.role not in ("home", "transit", "sample"):
            errors.append(f"node {node.id} has invalid role {node.role!r}")
        try:
            validate_q(node.q_rad, f"node {node.id} q_rad")
        except ValueError as exc:
            errors.append(str(exc))
            continue
        if limits is not None:
            for joint, (q, bounds) in enumerate(zip(node.q_rad, limits)):
                if q < bounds[0] or q > bounds[1]:
                    errors.append(
                        f"node {node.id} joint {joint}={q:.5f} outside "
                        f"[{bounds[0]:.5f}, {bounds[1]:.5f}]"
                    )
        if node.enabled and node.role == "home":
            homes.append(node)

    if len(homes) > 1:
        errors.append("exactly one enabled home is allowed")
    if require_home and len(homes) != 1:
        errors.append("exactly one enabled home is required")

    if homes and check_adjacent:
        ordered = [homes[0], *[n for n in plan.nodes if n.enabled and n.role != "home"]]
        for left, right in zip(ordered, ordered[1:]):
            delta = max(abs(a - b) for a, b in zip(left.q_rad, right.q_rad))
            if delta > plan.motion.max_adjacent_delta_rad:
                errors.append(
                    f"adjacent nodes {left.id}->{right.id} delta {delta:.5f} exceeds "
                    f"{plan.motion.max_adjacent_delta_rad:.5f}"
                )
        # 回程不再逐点倒退，最后一个节点直接回原点，这一跳同样受相邻限制
        if len(ordered) > 1:
            last = ordered[-1]
            delta = max(abs(a - b) for a, b in zip(last.q_rad, homes[0].q_rad))
            if delta > plan.motion.max_adjacent_delta_rad:
                errors.append(
                    f"return leg {last.id}->home {homes[0].id} delta {delta:.5f} exceeds "
                    f"{plan.motion.max_adjacent_delta_rad:.5f}; append transit nodes "
                    "after the last node"
                )
    if require_home and not any(n.enabled and n.role != "home" for n in plan.nodes):
        errors.append("at least one enabled transit or sample node is required")
    return errors


def route_for_plan(plan: Plan) -> list[tuple[PlanNode, str]]:
    """home → every enabled node in table order → straight back to home.

    The return leg is a single segment; ``validate_plan`` rejects it up front
    when the last node is too far from home, so the operator appends transit
    nodes instead of the arm retracing every sample pose."""
    errors = validate_plan(plan, require_home=True)
    if errors:
        raise ValueError("; ".join(errors))
    home = next(node for node in plan.nodes if node.enabled and node.role == "home")
    forward = [node for node in plan.nodes if node.enabled and node.role != "home"]
    return [
        (home, "forward"),
        *((node, "forward") for node in forward),
        (home, "reverse"),
    ]


def mirror_plan(plan: Plan, *, name: str | None = None) -> Plan:
    """Copy ``plan`` for the other arm with every node mirrored through the
    robot's sagittal plane. The copy is a draft: it keeps the mirrored home as a
    reference, but the operator must still place the arm there (start delta
    check) and re-validate before running."""
    target_arm = other_arm(plan.arm)
    copy = Plan.from_dict(plan.to_dict())
    copy.id = uuid.uuid4().hex[:12]
    copy.arm = target_arm
    copy.name = name or f"{plan.name} → {ARM_LABELS[target_arm]}"
    copy.draft = True
    copy.nodes = [
        PlanNode(
            id=node.id,
            name=node.name,
            role=node.role,
            q_rad=mirror_q(node.q_rad),
            enabled=node.enabled,
            source=f"mirror:{plan.arm}",
            metadata={**node.metadata, "mirrored_from": {"plan_id": plan.id, "arm": plan.arm}},
        )
        for node in plan.nodes
    ]
    copy.metadata = {**plan.metadata, "mirrored_from": {"plan_id": plan.id, "arm": plan.arm}}
    return copy
