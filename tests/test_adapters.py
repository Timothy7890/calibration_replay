import pytest

from calibration_replay.adapters import HttpCaptureAdapter


class FakeHttpAdapter(HttpCaptureAdapter):
    def __init__(self, responses, target="hand_eye_2D_head", **kwargs):
        super().__init__("http://fake", target, retries=0, **kwargs)
        self.responses = responses
        self.requests = []

    def _request(self, method, path, body=None):
        self.requests.append((method, path, body))
        response = self.responses[path]
        if isinstance(response, list):
            response = response.pop(0)
        if isinstance(response, Exception):
            raise response
        return response


def test_preflight_rejects_target_arm_control():
    adapter = FakeHttpAdapter(
        {
            "/api/status": {"ok": True},
            "/api/arm/status": {"enabled": True, "armed": False},
        }
    )
    with pytest.raises(RuntimeError, match="without --arm-control"):
        adapter.preflight("batch-01")


def test_2d_preflight_order_and_exact_capture_body():
    adapter = FakeHttpAdapter(
        {
            "/api/status": [
                {"arm": "right", "camera": {"serial": "old"}, "count": 0},
                {
                    "arm": "right",
                    "camera": {"serial": "CAM-22"},
                    "count": 0,
                    "run_id": "batch-01",
                },
            ],
            "/api/arm/status": {"available": False, "engaged": False},
            "/api/session/start": {
                "success": True, "run_id": "batch-01", "count": 0,
                "arm": "right", "save_path": "/data/runs/right/batch-01",
            },
            "/api/camera/select": {
                "success": True,
                "camera": {"serial": "CAM-22"},
            },
            "/api/checkerboard/detect": {
                "success": True,
                "found": False,
                "camera": {"serial": "CAM-22"},
            },
            "/api/capture": {"success": True, "index": 4},
        },
        camera_serial="CAM-22",
    )
    preflight = adapter.preflight("batch-01", record_dir="/data/runs/right/batch-01")
    assert preflight["detection"]["found"] is False
    assert [(method, path) for method, path, _ in adapter.requests] == [
        ("GET", "/api/status"),
        ("GET", "/api/arm/status"),
        ("POST", "/api/session/start"),
        ("POST", "/api/camera/select"),
        ("GET", "/api/status"),
        ("POST", "/api/checkerboard/detect"),
    ]
    assert adapter.requests[2][2] == {
        "run_id": "batch-01", "arm": "right", "camera_role": "head",
        "record_dir": "/data/runs/right/batch-01",
    }
    assert adapter.requests[3][2] == {"serial": "CAM-22", "camera_role": "head"}

    adapter = FakeHttpAdapter(
        {"/api/capture": {"success": True, "index": 4, "arm": "right",
                          "path": "/data/runs/right/batch-01/joints/0004.json"}}
    )
    result = adapter.capture(
        capture_id="stable-id",
        run_id="batch-01",
        waypoint_id="sample",
        target_q_rad=[0.1] * 7,
        stability={"stable": True},
        record_dir="/data/runs/right/batch-01",
    )
    assert result["index"] == 4
    method, path, payload = adapter.requests[-1]
    assert (method, path) == ("POST", "/api/capture")
    assert payload == {
        "run_id": "batch-01",
        "waypoint_id": "sample",
        "capture_id": "stable-id",
        "target_q_rad": [0.1] * 7,
        "stability": {"stable": True},
        "require_corners": True,
        "arm": "right",
    }


def test_2d_preflight_rejects_legacy_or_wrong_arm_service():
    """旧版 8131 没有 /api/session/start 且 /api/status 无 run_id → 预检失败；
    返回的 arm / save_path 与计划不符 → 失败，绝不把数据混进别的目录。"""
    legacy = FakeHttpAdapter(
        {
            "/api/status": [{"arm": "right", "count": 3}, {"arm": "right", "count": 3}],
            "/api/arm/status": {"available": False, "engaged": False},
            "/api/session/start": RuntimeError("POST /api/session/start returned HTTP 404"),
        }
    )
    with pytest.raises(RuntimeError, match="HTTP 404"):
        legacy.preflight("r1", record_dir="/data/runs/right/r1")

    wrong_arm = FakeHttpAdapter(
        {
            "/api/status": {"arm": "right", "count": 0, "recording": {"arm_selectable": True}},
            "/api/arm/status": {"available": False, "engaged": False},
            "/api/session/start": {"success": True, "run_id": "r1", "count": 0, "arm": "right",
                                   "save_path": "/data/runs/left/r1"},
        },
        arm="left",
    )
    with pytest.raises(RuntimeError, match="recording the right arm"):
        wrong_arm.preflight("r1", record_dir="/data/runs/left/r1")

    ignored_dir = FakeHttpAdapter(
        {
            "/api/status": {"arm": "right", "count": 0},
            "/api/arm/status": {"available": False, "engaged": False},
            "/api/session/start": {"success": True, "run_id": "r1", "count": 0, "arm": "right",
                                   "save_path": "/old/handeye_data/r1"},
        }
    )
    with pytest.raises(RuntimeError, match="ignored record_dir"):
        ignored_dir.preflight("r1", record_dir="/data/runs/right/r1")


def test_3d_exact_record_path_and_body():
    adapter = FakeHttpAdapter(
        {"/api/record/episode": {"ok": True, "episode": "episode_0001"}},
        target="hand_eye_3D",
        frame_count=7,
    )
    result = adapter.capture(
        capture_id="capture-3d",
        run_id="robot-4",
        waypoint_id="w2",
        target_q_rad=[0.2] * 7,
        stability={"stable": True, "sample_count": 30},
    )
    assert result["episode"] == "episode_0001"
    assert adapter.requests == [
        (
            "POST",
            "/api/record/episode",
            {
                "run_id": "robot-4",
                "waypoint_id": "w2",
                "capture_id": "capture-3d",
                "target_q_rad": [0.2] * 7,
                "stability": {"stable": True, "sample_count": 30},
                "frame_count": 7,
                "arm": "right",
            },
        )
    ]


def test_3d_capture_passes_record_dir_and_rejects_misplaced_episodes():
    adapter = FakeHttpAdapter(
        {"/api/record/episode": {"ok": True, "episode": "episode_0000", "path": "/data/runs/r1/episode_0000"}},
        target="hand_eye_3D",
    )
    result = adapter.capture(
        capture_id="c", run_id="r1", waypoint_id="w", target_q_rad=[0.0] * 7,
        stability={"stable": True}, record_dir="/data/runs/r1",
    )
    assert result["episode"] == "episode_0000"
    assert adapter.requests[0][2]["record_dir"] == "/data/runs/r1"

    legacy = FakeHttpAdapter(
        {"/api/record/episode": {"ok": True, "episode": "episode_0033", "path": "/old/teleop_data/biaoding/episode_0033"}},
        target="hand_eye_3D",
    )
    with pytest.raises(RuntimeError, match="ignored record_dir"):
        legacy.capture(
            capture_id="c", run_id="r1", waypoint_id="w", target_q_rad=[0.0] * 7,
            stability={"stable": True}, record_dir="/data/runs/r1",
        )
    assert len(legacy.requests) == 1  # 不重试，避免继续往错误目录写



def test_preflight_arm_rule_depends_on_capture_service_capability():
    """新版 8132 可按请求选臂 → 服务 --arm 不必等于计划臂；旧版必须一致。"""
    base = {"/api/arm/status": {"armed": False}}
    new_service = FakeHttpAdapter(
        {**base, "/api/status": {"arm": "right", "recording": {"arm_selectable": True}}},
        target="hand_eye_3D", arm="left",
    )
    assert new_service.preflight("r")["ok"]
    old_service = FakeHttpAdapter(
        {**base, "/api/status": {"arm": "right", "recording": {}}},
        target="hand_eye_3D", arm="left",
    )
    with pytest.raises(RuntimeError, match="restart it with --arm left"):
        old_service.preflight("r")

    # 录制结果里的 arm 与计划不一致 → 不重试直接失败
    wrong = FakeHttpAdapter(
        {"/api/record/episode": {"ok": True, "episode": "episode_0000", "arm": "right", "path": "/d/r/episode_0000"}},
        target="hand_eye_3D", arm="left",
    )
    with pytest.raises(RuntimeError, match="recorded the right arm instead of left"):
        wrong.capture(capture_id="c", run_id="r", waypoint_id="w", target_q_rad=[0.0] * 7,
                      stability={"stable": True}, record_dir="/d/r")
    assert len(wrong.requests) == 1


def test_2d_missing_corners_becomes_skip_not_fault():
    from calibration_replay.adapters import CaptureSkippedError, _CornersNotDetected

    # 8131 用 409 + corners_detected=false 拒绝；重试后仍没有 → 跳过（不是故障）
    adapter = FakeHttpAdapter(
        {"/api/capture": [_CornersNotDetected("未检出完整棋盘格"), _CornersNotDetected("未检出完整棋盘格")]},
    )
    adapter.retries = 1
    with pytest.raises(CaptureSkippedError, match="未检出完整棋盘格"):
        adapter.capture(
            capture_id="c1", run_id="r1", waypoint_id="n1", target_q_rad=[0.0] * 7,
            stability={}, record_dir=None,
        )
    assert len(adapter.requests) == 2

    # 不强制角点：8131 照样保存并返回 corners_detected=false，正常算一张
    lax = FakeHttpAdapter(
        {"/api/capture": {"success": True, "corners_detected": False, "arm": "right", "path": "/x/joints/0000.json"}},
        require_corners=False,
    )
    result = lax.capture(
        capture_id="c1", run_id="r1", waypoint_id="n1", target_q_rad=[0.0] * 7,
        stability={}, record_dir=None,
    )
    assert result["corners_detected"] is False
    assert lax.requests[0][2]["require_corners"] is False
