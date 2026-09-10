from __future__ import annotations

import json
import urllib.request

from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .adapters import HttpCaptureAdapter, MockCaptureAdapter
from .bridge import H2ArmBridge, MockArmBridge, read_urdf_limits
from .engine import ReplayEngine
from .exporter import build_route_trajectory, export_ik_replay
from .importer import import_3d_task, import_session, seed_default_imports

from .models import (
    ARM_LABELS, ARMS, Plan, PlanNode, best_insert_index, mirror_plan, validate_plan, validate_q,
)
from .storage import PlanStore


@dataclass
class AppConfig:
    data_root: str
    h2_project: str
    network_interface: str | None = None
    mock: bool = False
    # --mock 时仍通过 HTTP 调采集服务（对方也以 mock 启动）：全链路联调用
    mock_capture_http: bool = False
    base_url_2d: str = "http://127.0.0.1:8131"
    base_url_3d: str = "http://127.0.0.1:8132"
    # In-page 3D preview: URDF comes from the hand_eye_3D project, STL meshes
    # from a directory containing ``meshes/`` (defaults to IK_replay's H2 assets).
    robot_mesh_dir: str | None = None
    # 18000 capability registry (which arm/hand the robot currently has active).
    capability_url: str = "http://127.0.0.1:18000"


def arm_preview(arm: str) -> dict:
    """Viewer config for the driven arm; the other arm rests in a neutral pose."""
    other = "left" if arm == "right" else "right"
    sign = 1.0 if other == "left" else -1.0   # shoulder_roll mirrors between arms
    return {
        "arm": arm,
        "chain_id": f"{arm}_arm",
        "wrist_link": f"{arm}_wrist_yaw_link",
        "arm_links": [
            f"{arm}_shoulder_pitch_link", f"{arm}_shoulder_roll_link", f"{arm}_shoulder_yaw_link",
            f"{arm}_elbow_link", f"{arm}_wrist_roll_link", f"{arm}_wrist_pitch_link",
            f"{arm}_wrist_yaw_link", f"{arm}_hand_link",
        ],
        "initial_joints": {
            f"{other}_shoulder_pitch_joint": 0.2, f"{other}_shoulder_roll_joint": 0.25 * sign,
            f"{other}_elbow_joint": 0.9, f"{other}_wrist_pitch_joint": -0.1,
        },
    }


def fetch_capability(url: str, timeout_s: float = 1.5) -> dict:
    """Active arm/hand from the 18000 registry; unavailable is not an error."""
    try:
        request = urllib.request.Request(
            url.rstrip("/") + "/api/capability/registry", headers={"Accept": "application/json"}
        )
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            data = json.loads(response.read().decode("utf-8", errors="replace"))
    except Exception as exc:  # network, JSON, HTTP
        return {"available": False, "error": str(exc), "url": url}
    registry = data.get("registry") or data
    active = registry.get("active") or {}
    arm_name = str(active.get("arm") or "")
    hand_id = active.get("hand_id")
    hand = next((h for h in registry.get("hands", []) if h.get("id") == hand_id), {})
    return {
        "available": bool(active),
        "url": url,
        "arm": arm_name.removesuffix("_arm") or None,
        "hand_id": hand_id,
        "hand_name": hand.get("name") or hand_id,
        "active": active,
    }


def _resolve_mesh_dir(config: AppConfig) -> Path | None:
    candidates = [config.robot_mesh_dir] if config.robot_mesh_dir else [
        Path(config.h2_project).expanduser() / "assets/robots/h2",
        Path(config.h2_project).expanduser().parent.parent / "IK_replay/assets/robots/h2",
        Path("/home/robot/yx/project/IK_replay/assets/robots/h2"),
    ]
    for candidate in candidates:
        path = Path(candidate).expanduser() / "meshes"
        if path.is_dir() and any(path.glob("*.stl")):
            return path
    return None


def create_app(
    config: AppConfig,
    *,
    bridge=None,
    adapter_factory: Callable[[Plan], Any] | None = None,
) -> FastAPI:
    store = PlanStore(config.data_root)
    existing_targets = {plan.target for plan in store.list()}
    for name, target, url in (
        ("2D head", "hand_eye_2D_head", config.base_url_2d),
        ("2D waist", "hand_eye_2D_waist", config.base_url_2d),
        ("3D", "hand_eye_3D", config.base_url_3d),
    ):
        if target not in existing_targets:
            store.save(Plan.create(name, target, url))

    def limits_for(plan: Plan):
        if getattr(bridge, "arm", plan.arm) == plan.arm:
            return getattr(bridge, "limits", None)
        return None if config.mock else read_urdf_limits(config.h2_project, plan.arm)
    bridge = bridge or (
        MockArmBridge()
        if config.mock
        else H2ArmBridge(config.h2_project, config.network_interface)
    )
    if adapter_factory is None:
        adapter_factory = (
            (lambda _plan: MockCaptureAdapter())
            if config.mock and not config.mock_capture_http
            else lambda plan: HttpCaptureAdapter(
                plan.base_url,
                plan.target,
                arm=plan.arm,
                require_corners=False,  # 图像总是保存；缺角点的处置由引擎按 plan.on_missing_corners 决定
                camera_serial=plan.camera_serial,
            )
        )
    engine = ReplayEngine(
        bridge,
        adapter_factory,
        run_writer=store.write_run,
        run_dir_factory=store.create_run_dir,
        # --mock：不访问 18000/18089，也不等手回零
        hand_id_provider=(
            (lambda: "mock-hand") if config.mock
            else (lambda: fetch_capability(config.capability_url).get("hand_id"))
        ),
        hand_hold_settle_s=0.0 if config.mock else 1.5,
    )

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        yield
        if engine.status()["state"] in {
            "moving",
            "settling",
            "capturing",
            "returning",
            "paused",
        }:
            engine.immediate_stop()
        try:
            bridge.disarm()
        except Exception:
            pass

    app = FastAPI(
        title="H2 Calibration Trajectory Replay",
        version="0.1.0",
        description="Human-supervised calibration replay service; each plan drives one arm (left or right).",
        lifespan=lifespan,
    )
    urdf_path = Path(config.h2_project).expanduser() / "assets/robots/h2/robot.urdf"
    mesh_dir = _resolve_mesh_dir(config)
    app.state.config = config
    app.state.store = store
    app.state.bridge = bridge
    app.state.engine = engine

    def load(plan_id: str) -> Plan:
        try:
            return store.get(plan_id)
        except KeyError as exc:
            raise HTTPException(404, "plan not found") from exc

    def checked_save(plan: Plan) -> Plan:
        # 保存只做结构校验；相邻差值在校验/预览/运行时强制，便于先录原点再补过渡点
        errors = validate_plan(
            plan, limits=limits_for(plan), check_adjacent=False
        )
        if errors:
            raise HTTPException(422, detail=errors)
        plan.draft = len(
            [node for node in plan.nodes if node.enabled and node.role == "home"]
        ) != 1
        return store.save(plan)

    @app.get("/api/cameras/2d")
    def cameras_2d():
        """代理 8131 的相机枚举，给计划页的「2D 相机序列号」下拉用。"""
        url = config.base_url_2d.rstrip("/") + "/api/camera/devices"
        try:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(urllib.request.Request(url, headers={"Accept": "application/json"}), timeout=15) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as exc:  # noqa: BLE001 - 8131 没起来也要能回答
            return {"available": False, "devices": [], "current_serial": None, "last_error": f"8131 不可达: {exc}"}

    @app.get("/api/status")
    def status():
        return {
            **engine.status(),
            "mock": config.mock,
            "warning": "左臂/右臂由计划选择，必须全程人工监护。",
        }

    @app.get("/api/capability")
    def capability():
        return fetch_capability(config.capability_url)

    @app.get("/api/joints")
    def joints(arm: str | None = None):
        if arm is not None:
            if arm not in ARMS:
                raise HTTPException(422, f"arm must be left or right, got {arm!r}")
            try:
                bridge.select_arm(arm)
            except RuntimeError:
                pass  # other arm engaged: report that arm, UI shows the mismatch
        sample = bridge.read_sample()
        return {
            "ok": True,
            "arm": getattr(bridge, "arm", None),
            "joint_names": list(bridge.joint_names),
            "q": sample["q"],
            "dq": sample.get("dq"),
            "timestamp": sample.get("timestamp"),
            "source": bridge.source,
            # 编码器实测 vs 控制目标：/test 读数页用来看"按压时读数是否变化"
            "desired_rad": sample.get("desired_rad"),
            "cmd_rad": sample.get("cmd_rad"),
            "tau_est_nm": sample.get("tau_est_nm"),
            "engaged": bool(sample.get("engaged", False)),
        }

    @app.get("/api/plans")
    def list_plans():
        return {"plans": [plan.to_dict() for plan in store.list()]}

    @app.post("/api/plans")
    def create_plan(body: dict):
        try:
            plan = Plan.create(
                name=str(body["name"]),
                target=body["target"],
                base_url=str(body.get("base_url") or (
                    config.base_url_3d
                    if body["target"] == "hand_eye_3D"
                    else config.base_url_2d
                )),
                arm=str(body.get("arm") or "right"),
            )
            plan.camera_serial = (
                str(body["camera_serial"]).strip()
                if body.get("camera_serial")
                else None
            )
        except KeyError as exc:
            raise HTTPException(422, f"missing field {exc}") from exc
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        return checked_save(plan).to_dict()

    @app.post("/api/plans/{plan_id}/mirror")
    def mirror(plan_id: str, body: dict | None = None):
        """Copy the plan for the other arm (left↔right are symmetric peers)."""
        source = load(plan_id)
        copy = mirror_plan(source, name=(body or {}).get("name") or None)
        return checked_save(copy).to_dict()

    @app.get("/api/plans/{plan_id}")
    def get_plan(plan_id: str):
        return load(plan_id).to_dict()

    @app.put("/api/plans/{plan_id}")
    def update_plan(plan_id: str, body: dict):
        if str(body.get("id", plan_id)) != plan_id:
            raise HTTPException(409, "plan id cannot be changed")
        body["id"] = plan_id
        try:
            plan = Plan.from_dict(body)
        except (KeyError, TypeError, ValueError) as exc:
            raise HTTPException(422, str(exc)) from exc
        return checked_save(plan).to_dict()

    @app.delete("/api/plans/{plan_id}")
    def delete_plan(plan_id: str):
        try:
            store.delete(plan_id)
        except KeyError as exc:
            raise HTTPException(404, "plan not found") from exc
        return {"ok": True}

    @app.post("/api/plans/{plan_id}/nodes")
    def add_node(plan_id: str, body: dict):
        plan = load(plan_id)
        try:
            q = [float(value) for value in body["q_rad"]]
            validate_q(q)
            role = str(body.get("role", "sample"))
            if role not in ("home", "transit", "sample"):
                raise ValueError("role must be home, transit, or sample")
            node = PlanNode.create(
                str(body.get("name") or role),
                role,
                q,
                enabled=bool(body.get("enabled", True)),
                source="manual",
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise HTTPException(422, str(exc)) from exc
        if role == "home":
            plan.nodes = [item for item in plan.nodes if item.role != "home"]
            plan.nodes.insert(0, node)
        elif body.get("place") == "auto":
            # 自适应：插到绕路最小的缝隙（含回程缝），而不是追加到末尾
            plan.nodes.insert(best_insert_index(plan, q), node)
        else:
            plan.nodes.append(node)
        checked_save(plan)
        return node.__dict__

    @app.post("/api/plans/{plan_id}/nodes/{node_id}/autoplace")
    def autoplace_node(plan_id: str, node_id: str):
        """把已有节点挪到绕路最小的位置（原点固定在首位，不参与）。"""
        plan = load(plan_id)
        node = next((item for item in plan.nodes if item.id == node_id), None)
        if node is None:
            raise HTTPException(404, "node not found")
        if node.role == "home":
            raise HTTPException(422, "home stays first; it cannot be auto-placed")
        index = best_insert_index(plan, node.q_rad, exclude_id=node.id)
        plan.nodes = [item for item in plan.nodes if item.id != node.id]
        plan.nodes.insert(index, node)
        return checked_save(plan).to_dict()

    @app.post("/api/plans/{plan_id}/nodes/record")
    def record_node(plan_id: str, body: dict):
        plan = load(plan_id)
        try:
            bridge.select_arm(plan.arm)   # record the plan's arm, never the other one
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc
        body = dict(body)
        body["q_rad"] = bridge.read_sample()["q"]
        return add_node(plan_id, body)

    @app.patch("/api/plans/{plan_id}/nodes/{node_id}")
    def update_node(plan_id: str, node_id: str, body: dict):
        plan = load(plan_id)
        node = next((item for item in plan.nodes if item.id == node_id), None)
        if node is None:
            raise HTTPException(404, "node not found")
        for key in ("name", "role", "enabled", "q_rad"):
            if key in body:
                setattr(node, key, body[key])
        try:
            node.q_rad = [float(value) for value in node.q_rad]
            validate_q(node.q_rad)
            if node.role not in ("home", "transit", "sample"):
                raise ValueError("invalid node role")
        except (TypeError, ValueError) as exc:
            raise HTTPException(422, str(exc)) from exc
        return checked_save(plan).to_dict()

    @app.post("/api/plans/{plan_id}/nodes/reorder")
    def reorder_nodes(plan_id: str, body: dict):
        plan = load(plan_id)
        ids = list(body.get("ids", []))
        if set(ids) != {node.id for node in plan.nodes} or len(ids) != len(plan.nodes):
            raise HTTPException(422, "ids must contain every node exactly once")
        by_id = {node.id: node for node in plan.nodes}
        plan.nodes = [by_id[node_id] for node_id in ids]
        return checked_save(plan).to_dict()

    @app.delete("/api/plans/{plan_id}/nodes/{node_id}")
    def delete_node(plan_id: str, node_id: str):
        plan = load(plan_id)
        remaining = [node for node in plan.nodes if node.id != node_id]
        if len(remaining) == len(plan.nodes):
            raise HTTPException(404, "node not found")
        plan.nodes = remaining
        return checked_save(plan).to_dict()

    @app.post("/api/plans/{plan_id}/validate")
    def validate(plan_id: str):
        plan = load(plan_id)
        errors = validate_plan(
            plan,
            limits=limits_for(plan),
            require_home=True,
            require_capture_ready=True,
        )
        return {"ok": not errors, "errors": errors}

    @app.get("/api/plans/{plan_id}/preview")
    def preview(plan_id: str):
        plan = load(plan_id)
        try:
            trajectory = build_route_trajectory(plan, limits=limits_for(plan))
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        trajectory.pop("route", None)
        return {"ok": True, "plan_id": plan_id, **trajectory}

    @app.get("/api/robot/preview-config")
    def robot_preview_config(arm: str = "right"):
        if arm not in ARMS:
            raise HTTPException(422, f"arm must be left or right, got {arm!r}")
        return {
            "available": urdf_path.is_file() and mesh_dir is not None,
            "urdf_url": "/robot/robot.urdf" if urdf_path.is_file() else None,
            "mesh_base_url": "/robot/" if mesh_dir is not None else None,
            "urdf_path": str(urdf_path),
            "mesh_dir": str(mesh_dir) if mesh_dir is not None else None,
            **arm_preview(arm),
        }

    @app.get("/robot/robot.urdf")
    def robot_urdf():
        if not urdf_path.is_file():
            raise HTTPException(404, f"URDF not found: {urdf_path}")
        return FileResponse(urdf_path, media_type="application/xml")

    @app.post("/api/plans/{plan_id}/export")
    def export(plan_id: str, body: dict):
        plan = load(plan_id)
        try:
            return export_ik_replay(
                plan,
                body["output_dir"],
                limits=limits_for(plan),
            )
        except KeyError as exc:
            raise HTTPException(422, "output_dir is required") from exc
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc

    @app.post("/api/import/default-sessions")
    def import_defaults():
        plans = seed_default_imports(store, config.base_url_2d)
        return {"ok": True, "created": [plan.to_dict() for plan in plans]}

    @app.post("/api/import/session")
    def import_custom(body: dict):
        try:
            target = body["target"]
            if target == "hand_eye_3D":
                plan = import_3d_task(
                    body["session_dir"],
                    name=body["name"],
                    base_url=body.get("base_url", config.base_url_3d),
                    result_path=body.get("result_path") or None,
                )
            else:
                plan = import_session(
                    body["session_dir"],
                    target=target,
                    name=body["name"],
                    base_url=body.get("base_url", config.base_url_2d),
                )
        except (KeyError, OSError, TypeError, ValueError) as exc:
            raise HTTPException(422, str(exc)) from exc
        return checked_save(plan).to_dict()

    @app.post("/api/control/engage")
    def engage(body: dict | None = None):
        """Engage the arm of the given plan (or an explicit ``arm``)."""
        body = body or {}
        arm = body.get("arm")
        if body.get("plan_id"):
            arm = load(str(body["plan_id"])).arm
        if arm is not None and arm not in ARMS:
            raise HTTPException(422, f"arm must be left or right, got {arm!r}")
        try:
            engine.engage(arm)
            return {"ok": True, "arm": getattr(bridge, "arm", arm)}
        except Exception as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/control/guide")
    def guide():
        try:
            engine.guide()
            return {"ok": True}
        except Exception as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/control/catch")
    def catch():
        try:
            engine.catch_hold()
            return {"ok": True}
        except Exception as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/control/run/{plan_id}")
    def run(plan_id: str, body: dict | None = None):
        try:
            requested_run_id = (body or {}).get("run_id")
            run_id = engine.start(load(plan_id), requested_run_id)
            return {"ok": True, "run_id": run_id, "run_dir": engine.status().get("run_dir")}
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.get("/api/runs")
    def list_runs():
        return {"runs": store.list_runs(), "root": str(store.run_dir)}

    @app.post("/api/control/pause")
    def pause():
        try:
            engine.pause_after_current_node()
            return {"ok": True}
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/control/resume")
    def resume():
        try:
            engine.resume()
            return {"ok": True}
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/control/stop")
    def stop():
        engine.immediate_stop()
        return {"ok": True}

    @app.post("/api/control/disarm")
    def disarm():
        try:
            engine.disarm()
            return {"ok": True}
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc

    static_dir = Path(__file__).with_name("static")
    app.mount("/static", StaticFiles(directory=static_dir), name="static")
    if mesh_dir is not None:
        app.mount("/robot/meshes", StaticFiles(directory=mesh_dir), name="robot-meshes")

    @app.get("/")
    def index():
        return FileResponse(static_dir / "index.html")

    @app.get("/test")
    def joint_readout():
        """大屏关节读数：编码器实测 / 目标 / 力矩，带基准差值，用于按压、迟滞等物理测试。"""
        return FileResponse(static_dir / "test.html")

    return app
