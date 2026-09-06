import time
from pathlib import Path

import pytest

from calibration_replay.adapters import MockCaptureAdapter
from calibration_replay.bridge import MockArmBridge
from calibration_replay.engine import SAFE_RUN_ID_RE, ReplayEngine, safe_run_id
from calibration_replay.models import MotionConfig, Plan, PlanNode, StabilityConfig


class SlowBridge(MockArmBridge):
    def set_target(self, q):
        time.sleep(0.001)
        return super().set_target(q)


def make_plan():
    plan = Plan.create("engine", "hand_eye_2D_head", "http://unused")
    plan.camera_serial = "TEST-CAMERA"
    plan.nodes = [
        PlanNode("home", "home", "home", [0.0] * 7),
        PlanNode("sample", "sample", "sample", [0.12] * 7),
        PlanNode("transit", "transit", "transit", [0.18] * 7),
    ]
    plan.motion = MotionConfig(
        vmax_rad_s=1.0,
        amax_rad_s2=5.0,
        min_duration_s=0.02,
        max_adjacent_delta_rad=1.0,
        rate_hz=50.0,
    )
    plan.stability = StabilityConfig(
        window_s=0.02,
        max_error_rad=0.01,
        max_velocity_rad_s=0.01,
        max_range_rad=0.01,
        freshness_s=0.1,
        timeout_s=0.5,
    )
    return plan


def wait_state(engine, state, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if engine.status()["state"] == state:
            return True
        time.sleep(0.005)
    return False


def test_default_run_id_is_target_safe():
    generated = safe_run_id()
    assert SAFE_RUN_ID_RE.fullmatch(generated)
    assert generated.startswith("run_")
    named = safe_run_id(plan_name="右手3D-biaoding")
    assert SAFE_RUN_ID_RE.fullmatch(named)
    assert named.startswith("3D-biaoding_")
    with pytest.raises(ValueError):
        safe_run_id("有 空格")


def test_only_forward_sample_nodes_capture():
    bridge = MockArmBridge()
    adapter = MockCaptureAdapter()
    engine = ReplayEngine(bridge, lambda _plan: adapter, sleep=lambda _seconds: None)
    engine.engage()
    engine.start(make_plan(), "robot-07-head")
    assert engine.wait(2.0)
    assert engine.status()["state"] == "completed"
    assert adapter.preflight_calls == ["robot-07-head"]
    assert [call["waypoint_id"] for call in adapter.calls] == ["sample"]
    assert adapter.calls[0]["run_id"] == "robot-07-head"
    assert adapter.calls[0]["target_q_rad"] == [0.12] * 7
    assert adapter.calls[0]["stability"]["leg"] == "forward"
    assert bridge.read_sample()["q"] == [0.0] * 7


def test_pause_after_node_resume_and_immediate_stop():
    bridge = SlowBridge()
    adapter = MockCaptureAdapter()
    engine = ReplayEngine(bridge, lambda _plan: adapter, sleep=lambda _seconds: None)
    engine.engage()
    plan = make_plan()
    plan.nodes[1].q_rad = [0.6] * 7
    engine.start(plan, "pause-stop")
    assert wait_state(engine, "moving")
    engine.pause_after_current_node()
    assert wait_state(engine, "paused")
    engine.resume()
    assert wait_state(engine, "moving")
    engine.immediate_stop()
    assert engine.wait(2.0)
    status = engine.status()
    assert status["state"] == "stopped"
    assert status["arm"]["motion_enabled"] is False


def test_run_refuses_unplanned_move_from_pose_far_from_home():
    bridge = MockArmBridge()
    bridge._q = [0.4] * 7
    engine = ReplayEngine(bridge, lambda _plan: MockCaptureAdapter())
    engine.engage()

    try:
        engine.start(make_plan())
        raise AssertionError("far start pose should have been rejected")
    except RuntimeError as exc:
        assert "max_start_delta_rad" in str(exc)
    assert engine.status()["state"] == "armed"


def test_run_owns_a_directory_and_captures_record_into_it(tmp_path):
    from calibration_replay.storage import PlanStore

    store = PlanStore(tmp_path)
    bridge = MockArmBridge()
    adapter = MockCaptureAdapter()
    engine = ReplayEngine(
        bridge,
        lambda plan: adapter,
        run_writer=store.write_run,
        run_dir_factory=store.create_run_dir,
        sleep=lambda s: None,
    )
    engine.engage()
    run_id = engine.start(make_plan(), "trial-a")
    assert engine.wait(10)
    run_dir = tmp_path / "runs" / "trial-a"
    assert engine.status()["run_dir"] == str(run_dir)
    assert (run_dir / "run.json").is_file()
    assert adapter.calls[0]["record_dir"] == str(run_dir)
    assert store.list_runs()[0]["run_id"] == run_id
    with pytest.raises(ValueError, match="already used"):
        engine.start(make_plan(), "trial-a")


def test_capture_waits_configured_delay_after_stillness():
    bridge = MockArmBridge()
    adapter = MockCaptureAdapter()
    slept = []
    engine = ReplayEngine(bridge, lambda plan: adapter, sleep=lambda s: slept.append(s))
    plan = make_plan()
    plan.stability.capture_delay_s = 0.3
    engine.engage()
    engine.start(plan, "delay")
    assert engine.wait(10)
    assert engine.status()["state"] == "completed"
    assert any("等待 0.3s 后拍摄" in log["message"] for log in engine.status()["logs"])
    assert len(adapter.calls) == 1


def test_run_refuses_when_engaged_arm_differs_from_plan_arm():
    bridge = MockArmBridge()
    engine = ReplayEngine(bridge, lambda plan: MockCaptureAdapter(), sleep=lambda s: None)
    engine.engage("left")
    plan = make_plan()          # right-arm plan
    with pytest.raises(RuntimeError, match="right arm but the left arm is engaged"):
        engine.start(plan, "wrong-arm")
    with pytest.raises(RuntimeError, match="disarm before switching"):
        bridge.select_arm("right")
    engine.disarm()
    engine.engage("right")
    engine.start(plan, "right-arm")
    assert engine.wait(10) and engine.status()["state"] == "completed"
    assert engine.status()["arm"]["arm"] == "right"
