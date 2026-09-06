from __future__ import annotations

import json
import os
import threading
from pathlib import Path

from .models import ARMS, Plan

DEFAULT_PLANS = (
    ("2D head", "hand_eye_2D_head", "http://127.0.0.1:8131"),
    ("2D waist", "hand_eye_2D_waist", "http://127.0.0.1:8131"),
    ("3D", "hand_eye_3D", "http://127.0.0.1:8132"),
)


class PlanStore:
    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser().resolve()
        self.plan_dir = self.root / "plans"
        self.run_dir = self.root / "runs"
        self.plan_dir.mkdir(parents=True, exist_ok=True)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()

    def ensure_defaults(self) -> None:
        existing_targets = {plan.target for plan in self.list()}
        for name, target, url in DEFAULT_PLANS:
            if target not in existing_targets:
                self.save(Plan.create(name, target, url))

    def list(self) -> list[Plan]:
        with self._lock:
            plans: list[Plan] = []
            for path in sorted(self.plan_dir.glob("*.json")):
                try:
                    plans.append(Plan.from_dict(json.loads(path.read_text(encoding="utf-8"))))
                except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                    continue
            return plans

    def get(self, plan_id: str) -> Plan:
        path = self.plan_dir / f"{plan_id}.json"
        if not path.is_file():
            raise KeyError(plan_id)
        return Plan.from_dict(json.loads(path.read_text(encoding="utf-8")))

    def save(self, plan: Plan) -> Plan:
        path = self.plan_dir / f"{plan.id}.json"
        temp = path.with_suffix(".tmp")
        with self._lock:
            temp.write_text(
                json.dumps(plan.to_dict(), indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            os.replace(temp, path)
        return plan

    def delete(self, plan_id: str) -> None:
        path = self.plan_dir / f"{plan_id}.json"
        if not path.is_file():
            raise KeyError(plan_id)
        path.unlink()

    def run_path(self, run_id: str, arm: str) -> Path:
        """``runs/<left|right>/<run_id>/`` — the arm is the first layer so left
        and right data never share a directory."""
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
        return self.run_dir / arm / run_id

    def create_run_dir(self, run_id: str, arm: str) -> Path:
        """Every run owns its directory: the capture service writes its episodes
        there and ``run.json`` is written next to them. Refuses to reuse a name
        (within the same arm) so two runs can never mix their episodes."""
        path = self.run_path(run_id, arm)
        if path.exists():
            raise FileExistsError(f"run '{run_id}' already exists at {path}")
        path.mkdir(parents=True)
        return path

    def write_run(self, run_id: str, payload: dict) -> Path:
        arm = str((payload.get("plan") or {}).get("arm") or "right")
        run_dir = self.run_path(run_id, arm)
        run_dir.mkdir(parents=True, exist_ok=True)
        path = run_dir / "run.json"
        path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return path

    def list_runs(self) -> list[dict]:
        runs: list[dict] = []
        for path in sorted(self.run_dir.glob("*/*/run.json")):
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            runs.append(
                {
                    "run_id": payload.get("run_id", path.parent.name),
                    "arm": path.parent.parent.name,
                    "plan_name": (payload.get("plan") or {}).get("name"),
                    "outcome": payload.get("outcome"),
                    "started_at": payload.get("started_at"),
                    "finished_at": payload.get("finished_at"),
                    "capture_count": len(payload.get("captures") or []),
                    "episode_count": len(list(path.parent.glob("episode_*/data.json"))),
                    "path": str(path.parent),
                }
            )
        runs.sort(key=lambda r: r.get("started_at") or "", reverse=True)
        return runs
