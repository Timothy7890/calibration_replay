from fastapi.testclient import TestClient

from calibration_replay.adapters import MockCaptureAdapter
from calibration_replay.app import AppConfig, create_app


def test_plan_crud_record_validate_and_controls(tmp_path):
    adapter = MockCaptureAdapter()
    app = create_app(
        AppConfig(
            data_root=str(tmp_path / "data"),
            h2_project="/unused",
            mock=True,
        ),
        adapter_factory=lambda _plan: adapter,
    )
    with TestClient(app) as client:
        home = client.get("/")
        assert home.status_code == 200
        assert "左臂 / 右臂由计划选择" in home.text
        assert "必须全程人工监护" in home.text
        plans = client.get("/api/plans").json()["plans"]
        assert len(plans) == 3
        assert {
            plan["target"]: plan["base_url"] for plan in plans
        } == {
            "hand_eye_2D_head": "http://127.0.0.1:18005",
            "hand_eye_2D_waist": "http://127.0.0.1:18005",
            "hand_eye_3D": "http://127.0.0.1:18005/three-d",
        }
        response = client.post(
            "/api/plans",
            json={
                "name": "API plan",
                "target": "hand_eye_2D_head",
                "base_url": "http://capture",
                "camera_serial": "ROBOT-CAM-12",
            },
        )
        assert response.status_code == 200
        plan_id = response.json()["id"]
        assert client.post("/api/control/engage").status_code == 200
        assert client.post(
            f"/api/plans/{plan_id}/nodes/record",
            json={"name": "home", "role": "home"},
        ).status_code == 200
        assert client.post(
            f"/api/plans/{plan_id}/nodes",
            json={"name": "sample", "role": "sample", "q_rad": [0.1] * 7},
        ).status_code == 200
        validation = client.post(f"/api/plans/{plan_id}/validate").json()
        assert validation == {"ok": True, "errors": []}
        plan = client.get(f"/api/plans/{plan_id}").json()
        assert plan["draft"] is False
        assert plan["camera_serial"] == "ROBOT-CAM-12"
        plan["motion"].update(
            {
                "vmax_rad_s": 1.0,
                "amax_rad_s2": 5.0,
                "min_duration_s": 0.02,
                "rate_hz": 50.0,
            }
        )
        plan["stability"].update(
            {
                "window_s": 0.02,
                "max_error_rad": 0.01,
                "max_velocity_rad_s": 0.01,
                "max_range_rad": 0.01,
                "freshness_s": 0.1,
                "timeout_s": 0.5,
            }
        )
        assert client.put(f"/api/plans/{plan_id}", json=plan).status_code == 200
        run = client.post(
            f"/api/control/run/{plan_id}", json={"run_id": "robot-12-batch-a"}
        )
        assert run.json()["run_id"] == "robot-12-batch-a"
        assert app.state.engine.wait(2.0)
        assert adapter.preflight_calls == ["robot-12-batch-a"]
        assert adapter.calls[0]["run_id"] == "robot-12-batch-a"
        assert client.post(
            f"/api/control/run/{plan_id}", json={"run_id": "unsafe/id"}
        ).status_code == 409
        assert client.get("/api/joints").json()["source"] == "mock"
        assert client.post("/api/control/disarm").status_code == 200
        assert client.delete(f"/api/plans/{plan_id}").json() == {"ok": True}


def test_mirror_endpoint_and_arm_switch_rules(tmp_path):
    adapter = MockCaptureAdapter()
    app = create_app(
        AppConfig(data_root=str(tmp_path / "data"), h2_project="/unused", mock=True),
        adapter_factory=lambda _plan: adapter,
    )
    with TestClient(app) as client:
        created = client.post("/api/plans", json={"name": "L", "target": "hand_eye_3D", "arm": "left"}).json()
        assert created["arm"] == "left"
        node = client.post(f"/api/plans/{created['id']}/nodes", json={"role": "sample", "name": "s", "q_rad": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]}).json()
        mirrored = client.post(f"/api/plans/{created['id']}/mirror").json()
        assert mirrored["arm"] == "right" and mirrored["id"] != created["id"]
        assert mirrored["nodes"][0]["q_rad"] == [0.1, -0.2, -0.3, 0.4, -0.5, 0.6, -0.7]
        assert client.get("/api/robot/preview-config?arm=left").json()["chain_id"] == "left_arm"
        assert client.get("/api/joints?arm=left").json()["arm"] == "left"
        # 接管左臂后，右臂计划的录点被拒绝
        assert client.post("/api/control/engage", json={"plan_id": created["id"]}).json()["arm"] == "left"
        rejected = client.post(f"/api/plans/{mirrored['id']}/nodes/record", json={"role": "home", "name": "h"})
        assert rejected.status_code == 409
        assert client.post("/api/plans", json={"name": "bad", "target": "hand_eye_3D", "arm": "both"}).status_code == 422
