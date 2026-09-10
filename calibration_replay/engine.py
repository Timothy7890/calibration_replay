from __future__ import annotations

import hashlib
import re
import threading
import time
import uuid
from datetime import datetime, timezone
from typing import Any, Callable

from .adapters import CaptureSkippedError
from .models import ARM_LABELS, Plan, route_for_plan, validate_plan, validate_q
from .motion import interpolate_segment, segment_duration
from .stability import wait_for_stability

STATES = {
    "idle",
    "preflight",
    "armed",
    "moving",
    "settling",
    "capturing",
    "paused",
    "returning",
    "completed",
    "fault",
    "stopped",
}
SAFE_RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


def default_run_id(plan_name: str | None = None) -> str:
    """``<plan-slug>_<local time>``; the slug keeps only safe ASCII characters."""
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", str(plan_name or "")).strip("._-")
    slug = slug or "run"
    return f"{slug}_{datetime.now():%Y%m%d-%H%M%S}"


def safe_run_id(value: str | None = None, *, plan_name: str | None = None) -> str:
    run_id = (
        str(value).strip()
        if value is not None and str(value).strip()
        else default_run_id(plan_name)
    )
    if run_id in (".", "..") or not SAFE_RUN_ID_RE.fullmatch(run_id):
        raise ValueError(
            "run_id must be 1-128 safe characters: letters, digits, '.', '_' or '-'"
        )
    return run_id


class ReplayEngine:
    def __init__(
        self,
        bridge,
        adapter_factory: Callable[[Plan], Any],
        *,
        run_writer: Callable[[str, dict], Any] | None = None,
        run_dir_factory: Callable[[str], Any] | None = None,
        sleep: Callable[[float], None] = time.sleep,
        hand_id_provider: Callable[[], str | None] | None = None,
        hand_hold_settle_s: float = 1.5,
    ):
        self.bridge = bridge
        self.adapter_factory = adapter_factory
        self.run_writer = run_writer
        self.run_dir_factory = run_dir_factory
        self.sleep = sleep
        # 18000 当前激活的手型 id；plan.hold_hand_zero 时用它让 8132 经 18089 保持零位
        self.hand_id_provider = hand_id_provider
        self.hand_hold_settle_s = hand_hold_settle_s
        self._lock = threading.RLock()
        self._pause_condition = threading.Condition(self._lock)
        self._state = "idle"
        self._message = "左臂/右臂由计划选择；必须全程人工监护。"
        self._logs: list[dict[str, Any]] = []
        self._captures: list[dict[str, Any]] = []
        self._progress: dict[str, Any] = {}
        self._run_id: str | None = None
        self._run_dir: str | None = None
        self._stop = threading.Event()
        self._pause_after_node = False
        self._thread: threading.Thread | None = None

    def _set_state(self, state: str, message: str = "") -> None:
        if state not in STATES:
            raise ValueError(state)
        with self._lock:
            self._state = state
            if message:
                self._message = message
            self._logs.append(
                {
                    "at": datetime.now(timezone.utc).isoformat(),
                    "state": state,
                    "message": self._message,
                }
            )
            self._logs = self._logs[-300:]

    def status(self) -> dict[str, Any]:
        try:
            arm = self.bridge.status()
        except Exception as exc:
            arm = {"source": getattr(self.bridge, "source", "unknown"), "error": str(exc)}
        with self._lock:
            return {
                "state": self._state,
                "message": self._message,
                "run_id": self._run_id,
                "run_dir": self._run_dir,
                "progress": dict(self._progress),
                "captures": list(self._captures),
                "logs": list(self._logs[-100:]),
                "pause_requested": self._pause_after_node,
                "arm": arm,
            }

    def _arm_label(self) -> str:
        return ARM_LABELS.get(getattr(self.bridge, "arm", "right"), "手臂")

    def select_arm(self, arm: str) -> None:
        with self._lock:
            if self._state in {"moving", "settling", "capturing", "returning", "paused"}:
                raise RuntimeError("cannot switch arm during a run")
        self.bridge.select_arm(arm)

    def engage(self, arm: str | None = None) -> None:
        with self._lock:
            if self._state in {"moving", "settling", "capturing", "returning", "paused"}:
                raise RuntimeError("cannot engage during a run")
        self.bridge.engage(arm)
        self._set_state("armed", f"{self._arm_label()}已接管并保持；操作员必须留在控制位置。")

    def disarm(self) -> None:
        with self._lock:
            if self._state in {"moving", "settling", "capturing", "returning", "paused"}:
                raise RuntimeError("stop the run before disarming")
        label = self._arm_label()
        self.bridge.disarm()
        self._set_state("idle", f"已解除接管；交接期间请人工扶住{label}。")

    def guide(self) -> None:
        if not self.bridge.guide():
            raise RuntimeError("engage the arm before cooperative hand guide")
        self._set_state("armed", f"协作拖动已启用；请持续扶住{self._arm_label()}。")

    def catch_hold(self) -> None:
        self.bridge.catch_hold()
        self._set_state("armed", "已抓取并保持当前实测姿态。")

    def pause_after_current_node(self) -> None:
        with self._lock:
            if self._state not in {"moving", "settling", "capturing", "returning"}:
                raise RuntimeError("no active node to pause after")
            self._pause_after_node = True
            self._message = "已请求暂停；正在完成当前节点。"

    def resume(self) -> None:
        with self._pause_condition:
            if self._state != "paused":
                raise RuntimeError("run is not paused")
            self._pause_after_node = False
            self._pause_condition.notify_all()

    def immediate_stop(self) -> None:
        self._stop.set()
        self.bridge.stop_hold()
        with self._pause_condition:
            self._pause_after_node = False
            self._set_state("stopped", f"已立即中止轨迹并保持{self._arm_label()}。")
            self._pause_condition.notify_all()

    def start(self, plan: Plan, run_id: str | None = None) -> str:
        run_id = safe_run_id(run_id, plan_name=plan.name)
        errors = validate_plan(
            plan,
            limits=getattr(self.bridge, "limits", None),
            require_home=True,
            require_capture_ready=True,
        )
        if errors:
            raise ValueError("; ".join(errors))
        engaged_arm = self.bridge.status().get("arm")
        if engaged_arm is not None and engaged_arm != plan.arm:
            raise RuntimeError(
                f"plan is for the {plan.arm} arm but the {engaged_arm} arm is engaged; "
                "disarm and engage the plan's arm"
            )
        home = next(
            node for node in plan.nodes if node.enabled and node.role == "home"
        )
        current_q = [float(value) for value in self.bridge.read_sample()["q"]]
        validate_q(current_q, "current measured q")
        start_delta = max(
            abs(current - target)
            for current, target in zip(current_q, home.q_rad)
        )
        if start_delta > plan.motion.max_start_delta_rad:
            raise RuntimeError(
                f"current pose is {start_delta:.5f} rad from home, exceeding "
                f"motion.max_start_delta_rad={plan.motion.max_start_delta_rad:.5f}; "
                "use cooperative guide to place and catch the arm at home first"
            )
        with self._lock:
            if not self.bridge.status().get("engaged"):
                raise RuntimeError("engage the arm before preflight")
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("a run is already active")
            run_dir: str | None = None
            if self.run_dir_factory is not None:
                try:
                    run_dir = str(self.run_dir_factory(run_id, plan.arm))
                except FileExistsError as exc:
                    raise ValueError(f"run name already used: {exc}") from exc
            self._run_id = run_id
            self._run_dir = run_dir
            self._captures = []
            self._progress = {"plan_id": plan.id}
            self._pause_after_node = False
            self._set_state("preflight", "正在检查采集服务、会话和手臂控制权。")
        try:
            adapter = self.adapter_factory(plan)
            preflight = adapter.preflight(run_id, record_dir=run_dir)
        except Exception as exc:
            self._set_state("fault", f"预检拒绝运行：{exc}")
            raise RuntimeError(f"preflight failed: {exc}") from exc
        hand_hold = None
        if plan.hold_hand_zero and callable(getattr(adapter, "begin_hand_hold", None)):
            try:
                hand_id = self.hand_id_provider() if self.hand_id_provider else None
                hand_hold = adapter.begin_hand_hold(hand_id, plan.arm)
            except Exception as exc:
                self._set_state("fault", f"预检拒绝运行：灵巧手零位保持失败：{exc}")
                raise RuntimeError(f"hand hold failed: {exc}") from exc
        self._stop.clear()
        with self._lock:
            self._progress["preflight"] = preflight
            self._progress["hand_hold"] = hand_hold
        self.bridge.enable_motion()
        self._thread = threading.Thread(
            target=self._run,
            args=(plan, adapter, run_id, run_dir),
            name="calibration-replay",
            daemon=True,
        )
        self._thread.start()
        return run_id

    def _move(self, target: list[float], plan: Plan) -> None:
        start = self.bridge.read_sample()["q"]
        duration = segment_duration(
            start,
            target,
            vmax_rad_s=plan.motion.vmax_rad_s,
            amax_rad_s2=plan.motion.amax_rad_s2,
            min_duration_s=plan.motion.min_duration_s,
        )
        frames = interpolate_segment(
            start, target, duration_s=duration, rate_hz=plan.motion.rate_hz
        )
        period = 1.0 / plan.motion.rate_hz
        next_at = time.monotonic()
        for frame in frames:
            if self._stop.is_set():
                raise InterruptedError("run stopped")
            accepted = self.bridge.set_target(frame)
            if not accepted and self._stop.is_set():
                raise InterruptedError("run stopped")
            if not accepted:
                raise RuntimeError("controller rejected trajectory target")
            next_at += period
            self.sleep(max(0.0, next_at - time.monotonic()))

    def _sleep_interruptible(self, seconds: float) -> None:
        # 以 self.sleep 计时（而非墙钟），测试注入的假 sleep 才能让等待瞬间完成
        slept = 0.0
        while slept < seconds:
            if self._stop.is_set():
                raise InterruptedError("run stopped")
            step = min(0.05, seconds - slept)
            self.sleep(step)
            slept += step
        if self._stop.is_set():
            raise InterruptedError("run stopped")

    def _pause_if_requested(self, leg: str) -> None:
        with self._pause_condition:
            if not self._pause_after_node or self._stop.is_set():
                return
            self._set_state("paused", f"已在节点完成后安全暂停，{self._arm_label()}保持中。")
            while self._pause_after_node and not self._stop.is_set():
                self._pause_condition.wait(timeout=0.25)
            if self._stop.is_set():
                raise InterruptedError("run stopped while paused")
            self._set_state(
                "returning" if leg == "reverse" else "moving",
                "已在人工监护下继续运行。",
            )

    def _run(self, plan: Plan, adapter, run_id: str, run_dir: str | None = None) -> None:
        run_record: dict[str, Any] = {
            "run_id": run_id,
            "run_dir": run_dir,
            "plan": plan.to_dict(),
            "started_at": datetime.now(timezone.utc).isoformat(),
            "captures": [],
            "certificates": [],
        }
        try:
            run_record["hand_hold"] = self._progress.get("hand_hold")
            if run_record["hand_hold"] and self.hand_hold_settle_s > 0:
                # 给手指回零位一点时间，再开始动手臂
                self._set_state("preflight", "灵巧手正在回零位并保持…")
                self._sleep_interruptible(self.hand_hold_settle_s)
            route = route_for_plan(plan)
            sampling_aborted: str | None = None  # 触发"停止采样"的节点名
            for index, (node, leg) in enumerate(route):
                self._set_state(
                    "returning" if leg == "reverse" else "moving",
                    f"{'返回' if leg == 'reverse' else '前向'}移动至 {node.name}",
                )
                with self._lock:
                    self._progress.update(
                        {
                            "route_index": index,
                            "route_count": len(route),
                            "node_id": node.id,
                            "node_name": node.name,
                            "leg": leg,
                        }
                    )
                self._move(node.q_rad, plan)
                self._set_state("settling", f"正在确认 {node.name} 稳定到位")
                certificate = wait_for_stability(
                    self.bridge.read_sample,
                    node.q_rad,
                    plan.stability,
                    should_stop=self._stop.is_set,
                )
                certificate.update({"node_id": node.id, "leg": leg})
                run_record["certificates"].append(certificate)
                if not certificate.get("residual_within_reference", True):
                    # 规划值只是参考：偏差大只提醒，不阻止采集（采集端记录的是实测关节角）
                    self._set_state(
                        self._state,
                        f"{node.name} 已静止，但实测与规划参考偏差 "
                        f"{certificate['residual_max_rad']:.3f} rad（>"
                        f"{plan.stability.max_error_rad:.3f}），按实测继续",
                    )
                if leg == "forward" and node.role == "sample" and sampling_aborted:
                    # 已决定停止采样：剩余采样点当过渡点走，直到最后一个节点再回原点
                    self._set_state("moving", f"经过 {node.name}（已停止采样，正在沿剩余路径返回）")
                elif leg == "forward" and node.role == "sample":
                    delay = float(plan.stability.capture_delay_s)
                    if delay > 0:
                        self._set_state("settling", f"{node.name} 已静止，等待 {delay:.1f}s 后拍摄")
                        self._sleep_interruptible(delay)
                    self._set_state("capturing", f"正在采集 {node.name}")
                    capture_id = hashlib.sha256(
                        f"{run_id}:{node.id}".encode("utf-8")
                    ).hexdigest()[:24]
                    try:
                        result = adapter.capture(
                            capture_id=capture_id,
                            run_id=run_id,
                            waypoint_id=node.id,
                            target_q_rad=node.q_rad,
                            stability=certificate,
                            record_dir=run_dir,
                        )
                        corners = result.get("corners_detected")
                        item = {
                            "capture_id": capture_id,
                            "node_id": node.id,
                            "node_name": node.name,
                            "corners_detected": corners,
                            "result": result,
                        }
                        if corners is False:
                            if plan.on_missing_corners == "abort":
                                sampling_aborted = node.name
                                self._set_state(
                                    "capturing",
                                    f"{node.name} 未检出棋盘格（图像已保存）。按计划设置停止采样，"
                                    f"沿剩余过渡点返回原点",
                                )
                            else:
                                self._set_state("capturing", f"{node.name} 未检出棋盘格，图像已保存，继续下一点")
                    except CaptureSkippedError as exc:
                        # 这个点没拍到有用的东西（棋盘不在视野里），不算故障：记下、继续走后面的点
                        item = {
                            "capture_id": capture_id,
                            "node_id": node.id,
                            "node_name": node.name,
                            "skipped": True,
                            "reason": str(exc),
                            "result": None,
                        }
                        self._set_state("capturing", f"{node.name} 未检出棋盘格，跳过此点继续")
                    run_record["captures"].append(item)
                    with self._lock:
                        self._captures.append(item)
                        self._progress["captured"] = sum(1 for c in self._captures if not c.get("skipped"))
                        self._progress["no_corners"] = sum(
                            1 for c in self._captures if c.get("corners_detected") is False)
                        self._progress["skipped"] = sum(1 for c in self._captures if c.get("skipped"))
                        self._progress["sampling_aborted"] = sampling_aborted
                self._pause_if_requested(leg)
            self.bridge.stop_hold()
            run_record["finished_at"] = datetime.now(timezone.utc).isoformat()
            run_record["outcome"] = "completed"
            n_ok = sum(1 for c in run_record["captures"] if not c.get("skipped"))
            n_nc = sum(1 for c in run_record["captures"] if c.get("corners_detected") is False)
            n_skip = sum(1 for c in run_record["captures"] if c.get("skipped"))
            run_record["captured"] = n_ok
            run_record["no_corners"] = n_nc
            run_record["skipped"] = n_skip
            run_record["sampling_aborted_at"] = sampling_aborted
            summary = f"采集 {n_ok} 张"
            if n_nc:
                summary += f"，其中 {n_nc} 张未检出棋盘格"
            if sampling_aborted:
                summary += f"；在 {sampling_aborted} 停止采样并返回"
            if n_skip:
                summary += f"，{n_skip} 个点采集失败已跳过"
            self._set_state(
                "completed",
                f"计划完成（{summary}），{self._arm_label()}已安全返回原点。"
                + (f" 数据目录：{run_dir}" if run_dir else ""),
            )
        except InterruptedError:
            run_record["finished_at"] = datetime.now(timezone.utc).isoformat()
            run_record["outcome"] = "stopped"
            if self._state != "stopped":
                self.bridge.stop_hold()
                self._set_state("stopped", f"运行已停止，{self._arm_label()}保持中。")
        except Exception as exc:
            self.bridge.stop_hold()
            run_record["finished_at"] = datetime.now(timezone.utc).isoformat()
            run_record["outcome"] = "fault"
            run_record["error"] = str(exc)
            self._set_state("fault", f"故障：{exc}")
        finally:
            if run_record.get("hand_hold") and callable(getattr(adapter, "end_hand_hold", None)):
                try:
                    adapter.end_hand_hold()
                except Exception:
                    pass
            if self.run_writer is not None:
                self.run_writer(run_id, run_record)

    def wait(self, timeout: float | None = None) -> bool:
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()
