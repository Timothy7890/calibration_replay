import json

import pytest

from calibration_replay.exporter import export_ik_replay
from calibration_replay.importer import import_session
from calibration_replay.models import PlanNode


def make_session(root):
    (root / "joints").mkdir(parents=True)
    (root / "session_meta.json").write_text(
        json.dumps(
            {"camera": {"serial": "SOURCE-CAMERA", "name": "Do not infer target"}}
        ),
        encoding="utf-8",
    )
    (root / "handeye_result_left.json").write_text(
        json.dumps({"inlier_indices": [1]}), encoding="utf-8"
    )
    for index in range(3):
        (root / "joints" / f"{index:04d}.json").write_text(
            json.dumps({"index": index, "q_rad": [index * 0.1] * 7}),
            encoding="utf-8",
        )


def make_3d_task(root, arm="right", arms=None):
    from calibration_replay.models import joint_names_for

    for index in range(3):
        arm = (arms or [arm] * 3)[index]
        JOINT_NAMES = joint_names_for(arm)
        episode = root / f"episode_{index:04d}"
        episode.mkdir(parents=True)
        (episode / "data.json").write_text(
            json.dumps(
                {
                    "info": {
                        "kind": "hand_eye_calibration",
                        "arm": arm,
                        "joint_order": list(JOINT_NAMES),
                        "measured_q_rad": [index * 0.1] * 7,
                        "camera_serial": "CAM",
                    },
                    "data": [],
                }
            ),
            encoding="utf-8",
        )
    (root / "handeye3d_result.json").write_text(
        json.dumps({"pose_ids": ["episode_0001"]}), encoding="utf-8"
    )


def test_import_3d_task_uses_episode_joint_records(tmp_path):
    source = tmp_path / "task"
    make_3d_task(source)
    plan = import_session(
        source, target="hand_eye_3D", name="3d", base_url="http://capture"
    )
    assert plan.target == "hand_eye_3D"
    assert plan.draft is True
    assert plan.require_corners is False
    assert [node.id for node in plan.nodes] == [
        "episode_0000", "episode_0001", "episode_0002"
    ]
    assert [node.role for node in plan.nodes] == ["sample"] * 3
    assert plan.nodes[1].q_rad == [0.1] * 7

    from calibration_replay.importer import import_3d_task

    plan = import_3d_task(
        source,
        name="3d",
        base_url="http://capture",
        result_path=source / "handeye3d_result.json",
    )
    assert [node.role for node in plan.nodes] == ["transit", "sample", "transit"]
    assert all(node.enabled for node in plan.nodes)


def test_import_3d_task_takes_arm_from_episodes_and_rejects_mixed(tmp_path):
    source = tmp_path / "task"
    make_3d_task(source, arm="left")
    plan = import_session(source, target="hand_eye_3D", name="3d", base_url="http://capture")
    assert plan.arm == "left"
    assert plan.joint_names[0] == "left_shoulder_pitch_joint"

    mixed = tmp_path / "mixed"
    make_3d_task(mixed, arms=["right", "left", "right"])
    with pytest.raises(ValueError, match="one plan drives one arm"):
        import_session(mixed, target="hand_eye_3D", name="3d", base_url="http://capture")


def test_import_keeps_rejected_samples_as_enabled_transit_nodes(tmp_path):
    source = tmp_path / "session"
    make_session(source)
    plan = import_session(
        source,
        target="hand_eye_2D_head",
        name="imported",
        base_url="http://capture",
    )
    assert plan.draft is True
    assert [node.enabled for node in plan.nodes] == [True, True, True]
    assert [node.role for node in plan.nodes] == ["transit", "sample", "transit"]
    assert plan.metadata["imported_from"] == str(source.resolve())
    assert plan.camera_serial == "SOURCE-CAMERA"


def test_ik_export_writes_compatible_waypoints_and_sequence(tmp_path):
    source = tmp_path / "session"
    make_session(source)
    plan = import_session(
        source,
        target="hand_eye_2D_head",
        name="imported",
        base_url="http://capture",
    )
    plan.nodes.insert(0, PlanNode("home", "home", "home", [0.0] * 7))
    result = export_ik_replay(plan, tmp_path / "ik")
    sequence = json.loads(open(result["sequence"], encoding="utf-8").read())
    assert sequence["chain_id"] == "right_arm"
    assert sequence["waypoints"]
    trajectory = sequence["trajectory"]
    assert trajectory["frames"][0] == [0.0] * 7
    assert trajectory["frames"][-1] == [0.0] * 7
    assert trajectory["comparison_frames"] == trajectory["frames"]
    assert len(trajectory["execution_progress"]) == len(trajectory["frames"])
    waypoint_path = (
        tmp_path / "ik" / "data" / "waypoints" / sequence["waypoints"][0]
    )
    waypoint = json.loads(waypoint_path.read_text(encoding="utf-8"))
    assert waypoint["chain_id"] == "right_arm"
    assert len(waypoint["named_joints"]) == 7


def test_export_requires_home(tmp_path):
    source = tmp_path / "session"
    make_session(source)
    plan = import_session(
        source,
        target="hand_eye_2D_head",
        name="imported",
        base_url="http://capture",
    )
    with pytest.raises(ValueError, match="home"):
        export_ik_replay(plan, tmp_path / "ik")
