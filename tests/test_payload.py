"""末端负载重力补偿注入（payload_<arm>.json → H2ArmController._grav_model）。

不依赖 DDS / 作者 backend：用只带 ``torque()`` 的假基础模型，只验证读取、校验、替换与不叠加。
GravityWithPayload 本身来自 arm_payload_gravity，用一个最小替身模块代替，避免测试机没有该项目。
"""

from __future__ import annotations

import json
import sys
import types

import pytest
from fastapi.testclient import TestClient

from calibration_replay.app import AppConfig, create_app
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
