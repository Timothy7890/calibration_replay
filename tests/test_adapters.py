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
            "/api/session/start": {"success": True, "run_id": "batch-01", "count": 0},
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
    preflight = adapter.preflight("batch-01")
    assert preflight["detection"]["found"] is False
    assert [(method, path) for method, path, _ in adapter.requests] == [
        ("GET", "/api/status"),
        ("GET", "/api/arm/status"),
        ("POST", "/api/session/start"),
        ("POST", "/api/camera/select"),
        ("GET", "/api/status"),
        ("POST", "/api/checkerboard/detect"),
    ]
    assert adapter.requests[2][2] == {"run_id": "batch-01"}
    assert adapter.requests[3][2] == {"serial": "CAM-22"}

    adapter = FakeHttpAdapter({"/api/capture": {"success": True, "index": 4}})
    result = adapter.capture(
        capture_id="stable-id",
        run_id="batch-01",
        waypoint_id="sample",
        target_q_rad=[0.1] * 7,
        stability={"stable": True},
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
    }


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
