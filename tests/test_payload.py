"""末端负载重力补偿注入（payload_<arm>.json → H2ArmController._grav_model）。

不依赖 DDS / 作者 backend：用只带 ``torque()`` 的假基础模型，只验证读取、校验、替换与不叠加。
GravityWithPayload 本身来自 arm_payload_gravity，用一个最小替身模块代替，避免测试机没有该项目。
"""

from __future__ import annotations

import json
import sys
import types
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

import calibration_replay.app as replay_app
import calibration_replay.bridge as bridge_module
from calibration_replay.app import AppConfig, create_app, gravity_profile_catalog, resolve_gravity_profile
from calibration_replay.bridge import H2ArmBridge
from calibration_replay.payload import PayloadStore


class _Base:
    def torque(self, q, g_dir=None):
        return [1.0] * 7


class _Ctrl:
    def __init__(self):
        self._grav_model = _Base()


@pytest.fixture
def fake_gravity_project(tmp_path, monkeypatch):
    """假 arm_payload_gravity：gravity.ArmGravity / GravityWithPayload 只记录参数。"""
    proj = tmp_path / "apg"
    proj.mkdir()
    (proj / "gravity.py").write_text("", encoding="utf-8")
    mod = types.ModuleType("gravity")

    class ArmGravity:
        def __init__(self, arm):
            self.arm = arm

    class GravityWithPayload:
        def __init__(self, base, grav, mass, com, alpha):
            self.base, self.grav, self.mass, self.com, self.alpha = base, grav, mass, list(com), alpha

        def torque(self, q, g_dir=None):
            return [self.alpha * v + self.mass for v in self.base.torque(q, g_dir=g_dir)]

    mod.ArmGravity, mod.GravityWithPayload = ArmGravity, GravityWithPayload
    monkeypatch.setitem(sys.modules, "gravity", mod)
    return proj


def _write(dirpath, arm, **kw):
    """写 payload_<arm>.json；kw 覆盖字段（``arm_field`` 覆盖文件里的 arm，用于测不一致）。"""
    dirpath.mkdir(parents=True, exist_ok=True)
    data = {"mass_kg": 0.7, "com_m": [0.4, 0.0, -0.02], "alpha": 1.05, "arm": kw.pop("arm_field", arm),
            "applied_at": "2026-09-10T22:53:39", "source_session": "s1"}
    data.update(kw)
    (dirpath / f"payload_{arm}.json").write_text(json.dumps(data), encoding="utf-8")


def test_apply_replaces_model_and_reload_does_not_stack(tmp_path, fake_gravity_project):
    cfg = tmp_path / "config"
    _write(cfg, "right")
    store = PayloadStore(cfg, fake_gravity_project)
    ctrl = _Ctrl()
    base = ctrl._grav_model

    info = store.apply(ctrl, "right")
    assert info["active"] and info["mass_kg"] == 0.7 and info["alpha"] == 1.05
    assert ctrl._grav_model is not base and ctrl._grav_model.base is base
    assert ctrl._grav_model.torque([0.0] * 7) == pytest.approx([1.05 + 0.7] * 7)

    _write(cfg, "right", mass_kg=1.2, alpha=1.5)     # α 超上限 → 钳到 1.2
    info = store.apply(ctrl, "right")
    assert info["active"] and info["mass_kg"] == 1.2 and info["alpha"] == 1.2 and info["alpha_raw"] == 1.5
    assert ctrl._grav_model.base is base              # 以作者原模型为底，不是套娃


def test_apply_falls_back_when_missing_or_invalid(tmp_path, fake_gravity_project):
    cfg = tmp_path / "config"
    store = PayloadStore(cfg, fake_gravity_project)
    ctrl = _Ctrl()
    base = ctrl._grav_model

    info = store.apply(ctrl, "left")
    assert not info["active"] and "没有" in info["reason"] and ctrl._grav_model is base

    _write(cfg, "left", mass_kg=99.0)
    info = store.apply(ctrl, "left")
    assert not info["active"] and "不合理" in info["reason"] and ctrl._grav_model is base

    _write(cfg, "left", arm_field="right")            # 文件里 arm 不一致
    info = store.apply(ctrl, "left")
    assert not info["active"] and "不一致" in info["reason"] and ctrl._grav_model is base

    disabled = PayloadStore(None, None).apply(_Ctrl(), "right")
    assert disabled == {"enabled": False, "active": False, "arm": "right", "path": None, "reason": "已禁用（--no-payload）"}


def test_describe_file(tmp_path, fake_gravity_project):
    cfg = tmp_path / "config"
    store = PayloadStore(cfg, fake_gravity_project)
    assert store.describe_file("right")["exists"] is False
    _write(cfg, "right")
    d = store.describe_file("right")
    assert d["exists"] and d["mass_kg"] == 0.7 and d["path"].endswith("payload_right.json")


def test_api_payload_endpoints_mock(tmp_path):
    app = create_app(AppConfig(data_root=str(tmp_path / "data"), h2_project="/unused", mock=True))
    with TestClient(app) as client:
        info = client.get("/api/payload").json()
        assert info["ok"] and info["enabled"] is False and info["active"] is False
        assert client.post("/api/payload/reload").json()["active"] is False
        assert client.get("/api/status").json()["arm"]["payload"]["active"] is False


def test_resolve_gravity_profile_uses_18000_combo_default(monkeypatch):
    monkeypatch.setattr(replay_app, "fetch_capability", lambda _url: {
        "available": True,
        "active": {
            "arm": "left_arm", "hand_id": "qiangnao-revo2-left",
            "gravity_profile_version": "0.2.0",
        },
        "gravity_active_version": "0.1.0",
        "gravity_profiles": [
            {"version": "0.1.0", "label": "通用", "parameters": {
                "grav_alpha": 1.1, "payload_kg": 0.5,
                "grav_in_float": True, "use_imu_gravity": False,
            }},
            {"version": "0.2.0", "label": "左手", "compatibility": {
                "arm": "left_arm", "hand_id": "qiangnao-revo2-left",
            }, "parameters": {
                "grav_alpha": 1.0, "payload_kg": 0.766,
                "payload_com_m": [0.07, 0.01, 0.0],
                "grav_in_float": True, "use_imu_gravity": False,
            }},
        ],
    })

    selected = resolve_gravity_profile("http://18000", "left", None)
    assert selected["version"] == "0.2.0"
    left_catalog = gravity_profile_catalog("http://18000", "left")
    assert left_catalog["default_version"] == "0.2.0"
    assert [item["version"] for item in left_catalog["profiles"]] == ["0.1.0", "0.2.0"]
    right_catalog = gravity_profile_catalog("http://18000", "right")
    assert right_catalog["default_version"] == "0.1.0"
    assert [item["version"] for item in right_catalog["profiles"]] == ["0.1.0"]
    assert resolve_gravity_profile("http://18000", "left", "none")["mode"] == "base"
    with pytest.raises(ValueError, match="不适用于right臂"):
        resolve_gravity_profile("http://18000", "right", "0.2.0")


def test_h2_bridge_applies_selected_profile_before_start(monkeypatch):
    state = {}

    class Controller:
        def __init__(self, **kwargs):
            state["kwargs"] = kwargs
            self.joint_names = [f"j{i}" for i in range(7)]
            self.limits = SimpleNamespace(tolist=lambda: [[-1.0, 1.0]] * 7)

        def start(self):
            state["started"] = True

        def shutdown(self):
            state["stopped"] = True

    monkeypatch.setattr(
        bridge_module, "_module",
        lambda _project, name: SimpleNamespace(H2ArmController=Controller) if name == "arm" else None,
    )
    bridge = H2ArmBridge("/unused", None)
    bridge.engage("left", gravity_profile={
        "mode": "profile", "version": "0.2.0", "label": "左手",
        "parameters": {
            "grav_alpha": 1.0, "payload_kg": 0.766,
            "payload_com_m": [0.0774, 0.0106, -0.0069],
            "payload_link": "left_wrist_yaw_link",
            "excluded_subtree_link": "left_hand_link",
            "grav_in_float": True, "use_imu_gravity": False,
        },
    })

    assert state["started"] is True
    assert state["kwargs"]["payload_kg"] == 0.766
    assert state["kwargs"]["payload_com_m"] == [0.0774, 0.0106, -0.0069]
    assert bridge.payload_info()["version"] == "0.2.0"
    assert "file" not in bridge.payload_info()
    bridge.disarm()
    assert state["stopped"] is True
