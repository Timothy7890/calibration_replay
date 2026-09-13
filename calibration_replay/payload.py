"""末端负载重力补偿：把 arm_payload_gravity 标定出的 ``payload_<arm>.json`` 注入作者 H2ArmController。

arm_payload_gravity（10183）在多个静止姿态下辨识末端负载的等效质量 m、质心 c（末端连杆系）和
臂自重比例 α，「应用并保存」后写到 ``<payload_dir>/payload_<arm>.json``：

    {"mass_kg": 0.719, "com_m": [0.42, -0.005, -0.022], "alpha": 1.049, "arm": "right",
     "applied_at": "...", "source_session": "..."}

本模块只读该文件，用 arm_payload_gravity 的 ``GravityWithPayload`` 把作者控制器的 ``_grav_model``
换成「α·臂自重 + 负载项」。作者 ``_compute_tau`` 每周期读 ``self._grav_model`` 引用，Python 赋值原子，
接管中也能热替换。文件不存在 / 项目不存在时静默退回作者原前馈，并在 status 里说明原因。
"""

from __future__ import annotations

import importlib
import json
import sys
import time
from pathlib import Path
from typing import Any

ALPHA_MIN, ALPHA_MAX = 0.0, 1.2      # 与 arm_payload_gravity.PayloadArmController.set_payload 一致
MASS_MAX_KG = 5.0                    # 明显不合理的解不注入（读错文件 / 病态解）


class PayloadStore:
    """``payload_<arm>.json`` 读取 + ``GravityWithPayload`` 构造。``payload_dir=None`` 表示禁用。"""

    def __init__(self, payload_dir: str | Path | None, project: str | Path | None):
        self.payload_dir = Path(payload_dir).expanduser().resolve() if payload_dir else None
        self.project = Path(project).expanduser().resolve() if project else None
        self._gravity_mod = None

    @property
    def enabled(self) -> bool:
        return self.payload_dir is not None

    def path(self, arm: str) -> Path | None:
        return self.payload_dir / f"payload_{arm}.json" if self.payload_dir else None

    def load(self, arm: str) -> dict[str, Any] | None:
        """返回校验过的参数 dict（含 ``path``），没有文件返回 None，文件坏了抛 ValueError。"""
        path = self.path(arm)
        if path is None or not path.is_file():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        mass = float(data.get("mass_kg", 0.0))
        com = [float(v) for v in data.get("com_m", [0.0, 0.0, 0.0])]
        alpha = float(data.get("alpha", 1.0))
        if len(com) != 3:
            raise ValueError(f"{path}: com_m 需要 3 个数")
        if not (0.0 <= mass <= MASS_MAX_KG):
            raise ValueError(f"{path}: mass_kg={mass} 不合理（应在 0～{MASS_MAX_KG} kg）")
        if data.get("arm") not in (None, arm):
            raise ValueError(f"{path}: 文件里 arm={data.get('arm')!r} 与请求的 {arm!r} 不一致")
        return {
            "arm": arm,
            "mass_kg": mass,
            "com_m": com,
            "alpha": min(max(alpha, ALPHA_MIN), ALPHA_MAX),
            "alpha_raw": alpha,
            "applied_at": data.get("applied_at"),
            "source_session": data.get("source_session"),
            "path": str(path),
        }

    def _gravity(self):
        """惰性 import arm_payload_gravity/gravity.py（顶层模块名 gravity / urdf_fk，只读）。"""
        if self._gravity_mod is None:
            if self.project is None or not (self.project / "gravity.py").is_file():
                raise FileNotFoundError(f"arm_payload_gravity 项目不存在：{self.project}")
            if str(self.project) not in sys.path:
                sys.path.insert(0, str(self.project))
            self._gravity_mod = importlib.import_module("gravity")
        return self._gravity_mod

    def build_model(self, base_model, arm: str, params: dict[str, Any]):
        """``base_model`` 是作者控制器已有的 ``_grav_model``（ArmGravityModel），返回替换用的 GravityWithPayload。"""
        gravity = self._gravity()
        our = gravity.ArmGravity(arm)
        return gravity.GravityWithPayload(base_model, our, params["mass_kg"], params["com_m"], params["alpha"])

    def apply(self, controller, arm: str) -> dict[str, Any]:
        """把 ``payload_<arm>.json`` 注入 controller；返回 status 用的描述（active=True/False + reason）。"""
        info: dict[str, Any] = {"enabled": self.enabled, "active": False, "arm": arm,
                                "path": str(self.path(arm)) if self.enabled else None}
        if not self.enabled:
            info["reason"] = "已禁用（--no-payload）"
            return info
        try:
            params = self.load(arm)
        except (ValueError, OSError) as exc:
            info["reason"] = f"负载参数文件无效：{exc}"
            return info
        if params is None:
            info["reason"] = f"没有 {info['path']}：请先在 10183 标定并「应用并保存」"
            return info
        try:
            base = getattr(controller, "_payload_base_grav", None) or controller._grav_model
            if base is None:
                info["reason"] = "作者控制器没有重力模型（_grav_model=None）"
                return info
            model = self.build_model(base, arm, params)
        except Exception as exc:  # noqa: BLE001 - 注入失败必须退回原前馈而不是让接管失败
            info["reason"] = f"构造负载模型失败：{exc}"
            return info
        controller._payload_base_grav = base       # 记住作者原模型，重载时以它为底，避免叠加
        controller._grav_model = model             # 原子替换，控制线程下一周期生效
        info.update({k: params[k] for k in ("mass_kg", "com_m", "alpha", "alpha_raw", "applied_at", "source_session")})
        info.update({"active": True, "loaded_at": time.strftime("%Y-%m-%dT%H:%M:%S")})
        return info

    def describe_file(self, arm: str) -> dict[str, Any]:
        """不接管也能回答“磁盘上现在是什么参数”。"""
        info: dict[str, Any] = {"enabled": self.enabled, "arm": arm,
                                "path": str(self.path(arm)) if self.enabled else None, "exists": False}
        if not self.enabled:
            return info
        try:
            params = self.load(arm)
        except (ValueError, OSError) as exc:
            info["error"] = str(exc)
            return info
        if params is not None:
            info.update(params)
            info["exists"] = True
        return info
