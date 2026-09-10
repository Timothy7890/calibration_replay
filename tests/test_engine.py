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
    assert named.startswith("右手3D-biaoding_")      # 中文保留
    assert safe_run_id("头部标定-1") == "头部标定-1"
    with pytest.raises(ValueError):
        safe_run_id("有 空格")
    with pytest.raises(ValueError):
        safe_run_id("a/b")
    with pytest.raises(ValueError):
        safe_run_id(".hidden")


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
    run_dir = tmp_path / "runs" / "right" / "trial-a"
    assert engine.status()["run_dir"] == str(run_dir)
    assert (run_dir / "run.json").is_file()
    assert adapter.calls[0]["record_dir"] == str(run_dir)
    assert store.list_runs()[0]["run_id"] == run_id
    assert store.list_runs()[0]["arm"] == "right"
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


def test_hand_hold_starts_before_motion_and_stops_after_run():
    bridge = MockArmBridge()
    adapter = MockCaptureAdapter()
    engine = ReplayEngine(
        bridge, lambda _plan: adapter, sleep=lambda _s: None,
        hand_id_provider=lambda: "inspire-1-left",
    )
    engine.engage()
    plan = make_plan()
    assert plan.hold_hand_zero is True          # 默认开：标记贴在手上
    engine.start(plan, "hold-run")
    assert engine.wait(2.0)
    assert engine.status()["state"] == "completed"
    assert adapter.hand_hold_calls[0] == ("start", {"hand_id": "inspire-1-left", "side": plan.arm})
    assert adapter.hand_hold_calls[-1] == ("stop", None)
    assert engine.status()["progress"]["hand_hold"]["running"] is True

    # 关掉选项：完全不碰灵巧手
    adapter2 = MockCaptureAdapter()
    engine2 = ReplayEngine(bridge, lambda _plan: adapter2, sleep=lambda _s: None,
                           hand_id_provider=lambda: "inspire-1-left")
    plan2 = make_plan()
    plan2.hold_hand_zero = False
    engine2.start(plan2, "no-hold")
    assert engine2.wait(2.0)
    assert adapter2.hand_hold_calls == []


def test_hand_hold_failure_is_a_preflight_fault():
    class Refusing(MockCaptureAdapter):
        def begin_hand_hold(self, hand_id, side):
            raise RuntimeError("18089: 被视觉控制占用")

    bridge = MockArmBridge()
    engine = ReplayEngine(bridge, lambda _plan: Refusing(), sleep=lambda _s: None,
                          hand_id_provider=lambda: "inspire-1-left")
    engine.engage()
    with pytest.raises(RuntimeError, match="hand hold failed"):
        engine.start(make_plan(), "refused")
    assert engine.status()["state"] == "fault"
    assert bridge.read_sample()["q"] == [0.0] * 7     # 手臂没动


class NoCornersAdapter(MockCaptureAdapter):
    """8131 保存了图像但没检出棋盘格。"""

    def capture(self, **kwargs):
        result = super().capture(**kwargs)
        result["corners_detected"] = kwargs["waypoint_id"] != "s2"
        return result


def _three_sample_plan():
    plan = make_plan()
    plan.nodes = [
        PlanNode("home", "home", "home", [0.0] * 7),
        PlanNode("s1", "s1", "sample", [0.10] * 7),
        PlanNode("s2", "s2", "sample", [0.20] * 7),
        PlanNode("s3", "s3", "sample", [0.30] * 7),
        PlanNode("t", "t", "transit", [0.15] * 7),
    ]
    return plan


def test_missing_corners_continue_keeps_sampling():
    plan = _three_sample_plan()
    plan.on_missing_corners = "continue"
    adapter = NoCornersAdapter()
    engine = ReplayEngine(MockArmBridge(), lambda _plan: adapter, sleep=lambda _s: None)
    engine.engage()
    engine.start(plan, "r1")
    assert engine.wait(2.0)
    status = engine.status()
    assert status["state"] == "completed"
    assert [c["waypoint_id"] for c in adapter.calls] == ["s1", "s2", "s3"]
    assert [c["corners_detected"] for c in status["captures"]] == [True, False, True]
    assert status["progress"]["no_corners"] == 1
    assert "未检出棋盘格" in status["message"]


def test_missing_corners_abort_walks_rest_of_route_without_sampling():
    plan = _three_sample_plan()
    plan.on_missing_corners = "abort"
    adapter = NoCornersAdapter()
    bridge = MockArmBridge()
    engine = ReplayEngine(bridge, lambda _plan: adapter, sleep=lambda _s: None)
    engine.engage()
    engine.start(plan, "r2")
    assert engine.wait(2.0)
    status = engine.status()
    assert status["state"] == "completed"
    # s2 的图像已保存；s3 不再采样，但仍沿路径经过 s3、t 后回原点
    assert [c["waypoint_id"] for c in adapter.calls] == ["s1", "s2"]
    assert status["progress"]["sampling_aborted"] == "s2"
    assert bridge.read_sample()["q"] == [0.0] * 7
    assert "停止采样" in status["message"]


class SaggingBridge(MockArmBridge):
    """实测角比指令角"下垂"一点（有限 kp + 重力），并记录下发的轨迹。"""

    SAG = 0.05

    def __init__(self):
        super().__init__()
        self.sent = []

    def set_target(self, q):
        self.sent.append(list(q))
        return super().set_target(q)

    def read_sample(self):
        sample = super().read_sample()
        sample["cmd_rad"] = list(sample["q"])
        sample["q"] = [v - self.SAG for v in sample["q"]]
        return sample


def test_move_starts_from_last_command_not_measured():
    bridge = SaggingBridge()
    engine = ReplayEngine(bridge, lambda _plan: MockCaptureAdapter(), sleep=lambda _seconds: None)
    engine.engage()
    engine.start(make_plan(), "sag")
    assert engine.wait(2.0)
    assert engine.status()["state"] == "completed"
    # 每段第一帧都等于上一条指令角，绝不会被拉回到下垂的实测角
    assert all(min(frame) >= 0.0 for frame in bridge.sent)
    assert bridge.sent[0] == [0.0] * 7
