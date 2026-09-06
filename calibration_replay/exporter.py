from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path

from .models import Plan, route_for_plan, validate_plan
from .motion import interpolate_segment, segment_duration


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip()).strip("._")
    return cleaned or "waypoint"


def build_route_trajectory(
    plan: Plan,
    *,
    limits: list[list[float]] | None = None,
) -> dict:
    """Interpolate the full home→…→home route exactly as the engine will run it.

    Returns frames at ``plan.motion.rate_hz`` plus, for every route stop, the
    frame index where the arm is at that node (used by the in-page preview).
    """
    errors = validate_plan(plan, limits=limits, require_home=True)
    if errors:
        raise ValueError("; ".join(errors))
    route = route_for_plan(plan)
    frames: list[list[float]] = [list(route[0][0].q_rad)]
    stops: list[dict] = [
        {
            "node_id": route[0][0].id,
            "name": route[0][0].name,
            "role": route[0][0].role,
            "leg": route[0][1],
            "frame_index": 0,
            "time_s": 0.0,
        }
    ]
    duration_s = 0.0
    for (left, _), (right, leg) in zip(route, route[1:]):
        duration = segment_duration(
            left.q_rad,
            right.q_rad,
            vmax_rad_s=plan.motion.vmax_rad_s,
            amax_rad_s2=plan.motion.amax_rad_s2,
            min_duration_s=plan.motion.min_duration_s,
        )
        duration_s += duration
        segment = list(
            interpolate_segment(
                left.q_rad,
                right.q_rad,
                duration_s=duration,
                rate_hz=plan.motion.rate_hz,
            )
        )
        frames.extend(segment[1:])
        stops.append(
            {
                "node_id": right.id,
                "name": right.name,
                "role": right.role,
                "leg": leg,
                "frame_index": len(frames) - 1,
                "time_s": duration_s,
            }
        )
    return {
        "frames": frames,
        "joint_names": list(plan.joint_names),
        "duration_s": duration_s,
        "rate_hz": plan.motion.rate_hz,
        "stops": stops,
        "route": route,
    }


def export_ik_replay(
    plan: Plan,
    output_dir: str | Path,
    *,
    limits: list[list[float]] | None = None,
) -> dict:
    trajectory = build_route_trajectory(plan, limits=limits)
    route = trajectory["route"]
    root = Path(output_dir).expanduser().resolve()
    waypoint_dir = root / "data" / "waypoints"
    sequence_dir = root / "data" / "sequences"
    waypoint_dir.mkdir(parents=True, exist_ok=True)
    sequence_dir.mkdir(parents=True, exist_ok=True)
    created_at = datetime.now(timezone.utc).isoformat()

    refs: list[str] = []
    exported_nodes: set[str] = set()
    for node, _leg in route:
        filename = f"{_safe_name(plan.id)}-{_safe_name(node.id)}.json"
        refs.append(filename)
        if node.id in exported_nodes:
            continue
        exported_nodes.add(node.id)
        payload = {
            "name": node.name,
            "chain_id": f"{plan.arm}_arm",
            "named_joints": dict(zip(plan.joint_names, node.q_rad)),
            "created_at": created_at,
            "planner": {
                "source": "calibration_replay",
                "plan_id": plan.id,
                "plan_version": plan.version,
                "node_id": node.id,
                "role": node.role,
            },
        }
        (waypoint_dir / filename).write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    frames = trajectory["frames"]
    duration_s = trajectory["duration_s"]
    divisor = max(1, len(frames) - 1)
    progress = [index / divisor for index in range(len(frames))]
    sequence_name = f"{_safe_name(plan.id)}-{_safe_name(plan.name)}.json"
    sequence = {
        "name": plan.name,
        "chain_id": f"{plan.arm}_arm",
        "waypoints": refs,
        "created_at": created_at,
        "trajectory": {
            "frames": frames,
            "comparison_frames": frames,
            "comparison_progress": progress,
            "execution_progress": progress,
            "duration_s": duration_s,
            "joint_names": list(plan.joint_names),
            "recorded_at": created_at,
            "planner": "calibration-replay-quintic-50hz",
            "planner_metadata": {
                "source_plan_id": plan.id,
                "source_plan_version": plan.version,
                "authoritative_format": "calibration_replay plan JSON",
                "route": "home-forward-home",
                "vmax_rad_s": plan.motion.vmax_rad_s,
                "amax_rad_s2": plan.motion.amax_rad_s2,
                "max_start_delta_rad": plan.motion.max_start_delta_rad,
                "rate_hz": plan.motion.rate_hz,
            },
        },
    }
    sequence_path = sequence_dir / sequence_name
    sequence_path.write_text(
        json.dumps(sequence, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return {
        "output_dir": str(root),
        "sequence": str(sequence_path),
        "waypoints": len(exported_nodes),
        "frames": len(frames),
        "duration_s": duration_s,
    }
