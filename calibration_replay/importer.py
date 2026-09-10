from __future__ import annotations

import json
from pathlib import Path

from .models import ARMS, JOINT_NAMES, Plan, PlanNode, joint_names_for

DEFAULT_SESSIONS = {
    "hand_eye_2D_head": Path(
        "/home/robot/yx/project/calib/hand_eye_2D/handeye_data/20260902_170106"
    ),
    "hand_eye_2D_waist": Path(
        "/home/robot/yx/project/calib/hand_eye_2D/handeye_data/20260902_173157"
    ),
}
DEFAULT_LABELS = {
    "hand_eye_2D_head": "Imported 2D head",
    "hand_eye_2D_waist": "Imported 2D waist",
}


def import_session(
    session_dir: str | Path,
    *,
    target: str,
    name: str,
    base_url: str,
) -> Plan:
    if target == "hand_eye_3D":
        return import_3d_task(session_dir, name=name, base_url=base_url)
    root = Path(session_dir).expanduser().resolve()
    result_path = root / "handeye_result_left.json"
    joints_dir = root / "joints"
    if not result_path.is_file() or not joints_dir.is_dir():
        raise ValueError(f"{root} is not a solved hand_eye_2D session")
    result = json.loads(result_path.read_text(encoding="utf-8"))
    meta_path = root / "session_meta.json"
    session_meta = (
        json.loads(meta_path.read_text(encoding="utf-8"))
        if meta_path.is_file()
        else {}
    )
    inliers = {int(value) for value in result.get("inlier_indices", [])}
    plan = Plan.create(name=name, target=target, base_url=base_url)
    session_arm = str(session_meta.get("arm") or "").strip()
    if session_arm:
        if session_arm not in ARMS:
            raise ValueError(f"{root} session_meta.json 的 arm={session_arm!r} 不是 left/right")
        plan.arm = session_arm
    camera_serial = (session_meta.get("camera") or {}).get("serial")
    plan.camera_serial = str(camera_serial).strip() if camera_serial else None
    plan.require_corners = True
    plan.draft = True
    plan.metadata = {
        "imported_from": str(root),
        "solver_inlier_indices": sorted(inliers),
        "note": "Draft: record or set home before validation/run.",
    }
    for path in sorted(joints_dir.glob("*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        index = int(record.get("index", int(path.stem)))
        q = [float(value) for value in record["q_rad"]]
        plan.nodes.append(
            PlanNode(
                id=f"sample-{index:04d}",
                name=(
                    f"sample {index:04d}"
                    if index in inliers
                    else f"transit from rejected sample {index:04d}"
                ),
                role="sample" if index in inliers else "transit",
                q_rad=q,
                enabled=True,
                source="hand_eye_2D_import",
                metadata={
                    "session": str(root),
                    "original_index": index,
                    "solver_inlier": index in inliers,
                    "original_joint_file": str(path),
                },
            )
        )
    if not plan.nodes:
        raise ValueError(f"{root} contains no joint records")
    return plan


def _episode_q(info: dict, label: str, joint_names: list[str] = JOINT_NAMES) -> list[float]:
    raw = info.get("measured_q_rad")
    if raw is None:
        raw = (info.get("measured_q_summary") or {}).get("median_rad")
    if raw is None:
        raise ValueError(f"{label} has no measured_q_rad")
    q = [float(value) for value in raw]
    if len(q) != len(joint_names):
        raise ValueError(f"{label} joint vector has {len(q)} values, expected {len(joint_names)}")
    joint_order = info.get("joint_order")
    if joint_order is not None:
        # hand_eye_3D episodes store dataset names without the URDF "_joint" suffix.
        normalized = [str(n).removesuffix("_joint") for n in joint_order]
        expected = [n.removesuffix("_joint") for n in joint_names]
        if normalized != expected:
            raise ValueError(f"{label} joint_order {joint_order} does not match {joint_names}")
    return q


def import_3d_task(
    task_dir: str | Path,
    *,
    name: str,
    base_url: str,
    result_path: str | Path | None = None,
) -> Plan:
    """Import a hand_eye_3D episode directory (``episode_*/data.json``).

    Every episode becomes an enabled ``sample`` node from its recorded
    ``info.measured_q_rad``. When a solved ``handeye3d_result.json`` is
    supplied, episodes absent from its ``pose_ids`` become ``transit`` nodes
    (known reached poses, but not worth re-capturing). The plan's arm is taken
    from the episodes (``info.arm``, default right); a directory mixing both arms
    is rejected.
    """
    root = Path(task_dir).expanduser().resolve()
    episodes = sorted(root.glob("episode_*/data.json"))
    if not episodes:
        raise ValueError(f"{root} contains no hand_eye_3D episode_*/data.json")
    arm: str | None = None
    pose_ids: set[str] | None = None
    if result_path is not None:
        result = json.loads(Path(result_path).expanduser().read_text(encoding="utf-8"))
        pose_ids = {str(value) for value in result.get("pose_ids", [])}

    plan = Plan.create(name=name, target="hand_eye_3D", base_url=base_url)
    plan.camera_serial = None
    plan.require_corners = False
    plan.draft = True
    plan.metadata = {
        "imported_from": str(root),
        "solver_pose_ids": sorted(pose_ids) if pose_ids is not None else None,
        "note": "Draft: record or set home before validation/run.",
    }
    for path in episodes:
        episode = path.parent.name
        payload = json.loads(path.read_text(encoding="utf-8"))
        info = payload.get("info") or {}
        if str(info.get("kind", "hand_eye_calibration")) != "hand_eye_calibration":
            continue
        episode_arm = str(info.get("arm") or "right")
        if episode_arm not in ARMS:
            raise ValueError(f"{episode} has unsupported arm {episode_arm!r}")
        if arm is None:
            arm = episode_arm
            plan.arm = arm
        elif episode_arm != arm:
            raise ValueError(
                f"{episode} was recorded with the {episode_arm} arm but earlier episodes "
                f"use the {arm} arm; one plan drives one arm"
            )
        q = _episode_q(info, episode, joint_names_for(arm))
        is_sample = pose_ids is None or episode in pose_ids
        plan.nodes.append(
            PlanNode(
                id=episode,
                name=episode if is_sample else f"transit from unused {episode}",
                role="sample" if is_sample else "transit",
                q_rad=q,
                enabled=True,
                source="hand_eye_3D_import",
                metadata={
                    "task_dir": str(root),
                    "episode": episode,
                    "data_json": str(path),
                    "created_at": info.get("created_at"),
                    "camera_serial": info.get("camera_serial"),
                    "solver_pose": episode in pose_ids if pose_ids is not None else None,
                },
            )
        )
    if not plan.nodes:
        raise ValueError(f"{root} contains no hand_eye_calibration episodes")
    return plan


def seed_default_imports(store, base_url: str) -> list[Plan]:
    created: list[Plan] = []
    existing_sources = {
        plan.metadata.get("imported_from") for plan in store.list() if plan.metadata
    }
    for target, source in DEFAULT_SESSIONS.items():
        if str(source.resolve()) in existing_sources:
            continue
        label = DEFAULT_LABELS[target]
        plan = import_session(source, target=target, name=label, base_url=base_url)
        store.save(plan)
        created.append(plan)
    return created
