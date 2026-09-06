"""Arrival detection based on the arm's own state, not on the planned target.

Mirrors the practice in IK_replay's reach service: first wait until the
rate-limited controller has actually delivered the final command
(``desired ≈ cmd``), then require the *measured* state to be still for a
continuous window — encoder drift inside the window, window-averaged encoder
velocity and window-averaged torso IMU angular rate all below their thresholds
(averaging because the motor's dq report is quantized noise of up to ±0.08 rad/s
under gravity load while the encoder position does not move at all). The planned joint vector is
only a reference: the deviation between measured and planned is reported in the
certificate (and flagged when large) but never blocks arrival, because the
capture service records the live joint readings anyway.
"""

from __future__ import annotations

import math
import time
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import asdict
from typing import Any

from .models import StabilityConfig

COMMAND_DELIVERED_GAP_RAD = 1e-3


def _gyro_vector(sample: dict[str, Any]) -> list[float] | None:
    gyro = sample.get("gyro_rad_s")
    if gyro is None:
        return None
    try:
        values = [float(v) for v in gyro]
    except (TypeError, ValueError):
        return None
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        return None
    return values


def wait_for_stability(
    read_sample: Callable[[], dict[str, Any]],
    target: Sequence[float],
    config: StabilityConfig,
    *,
    should_stop: Callable[[], bool] = lambda: False,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    poll_s: float = 0.02,
) -> dict[str, Any]:
    started = clock()
    target_q = [float(v) for v in target]
    window: deque[dict[str, Any]] = deque()
    previous: dict[str, Any] | None = None
    command_delivered_at: float | None = None
    gyro_seen = False
    last_diag: dict[str, Any] | None = None

    while clock() - started <= config.timeout_s:
        if should_stop():
            raise InterruptedError("stability wait stopped")
        now = clock()
        sample = read_sample()
        q = [float(v) for v in sample["q"]]
        timestamp = float(sample.get("timestamp", now))

        # ---- 阶段 1：控制器是否已把最终指令送达（限速滑动结束） ----
        if command_delivered_at is None:
            gap = sample.get("cmd_gap_rad")
            elapsed = now - started
            if gap is None or float(gap) <= COMMAND_DELIVERED_GAP_RAD:
                command_delivered_at = now
            elif elapsed >= config.command_settle_s:
                # 指令一直送不完（例如被限速拖住）也不再等，转而只看自身是否静止
                command_delivered_at = now
            else:
                previous = {"q": q, "timestamp": timestamp}
                sleep(poll_s)
                continue

        # ---- 阶段 2：自身静止（编码器 + IMU），与规划值无关 ----
        # 电机上报的 dq 在大重力载荷下量化噪声可达 ±0.08 rad/s（关节位置却一格不动），
        # 所以速度和 IMU 都按窗口内的均值判：噪声互相抵消，真实运动不会。
        dq = sample.get("dq")
        if dq is None and previous is not None:
            dt = timestamp - previous["timestamp"]
            dq = (
                [(a - b) / dt for a, b in zip(q, previous["q"])]
                if dt > 0
                else [0.0] * len(q)
            )
        elif dq is None:
            dq = [0.0] * len(q)
        dq = [float(v) for v in dq]
        previous = {"q": q, "timestamp": timestamp}
        gyro = _gyro_vector(sample)
        gyro_seen = gyro_seen or gyro is not None

        if now - timestamp > config.freshness_s:
            window.clear()   # 数据断流：之前的窗口不能证明现在仍静止
            sleep(poll_s)
            continue
        window.append(
            {"timestamp": timestamp, "q": q, "dq": dq, "gyro": gyro, "age_s": max(0.0, now - timestamp)}
        )
        cutoff = now - config.window_s
        # Keep the one sample immediately before the cutoff so a sampled
        # signal can prove a full continuous window despite scheduler jitter.
        while len(window) > 1 and window[1]["timestamp"] <= cutoff:
            window.popleft()
        if window[-1]["timestamp"] - window[0]["timestamp"] < config.window_s:
            sleep(poll_s)
            continue

        n = len(window)
        ranges = [
            max(row["q"][j] for row in window) - min(row["q"][j] for row in window)
            for j in range(len(target_q))
        ]
        mean_dq = [abs(sum(row["dq"][j] for row in window) / n) for j in range(len(target_q))]
        gyros = [row["gyro"] for row in window if row["gyro"] is not None]
        mean_gyro = (
            math.sqrt(sum((sum(g[k] for g in gyros) / len(gyros)) ** 2 for k in range(3)))
            if gyros
            else None
        )
        last_diag = {
            "joint_range_rad": ranges,
            "mean_velocity_rad_s": mean_dq,
            "mean_gyro_rad_s": mean_gyro,
        }
        if (
            max(ranges) <= config.max_range_rad
            and max(mean_dq) <= config.max_velocity_rad_s
            and (mean_gyro is None or mean_gyro <= config.max_gyro_rad_s)
        ):
            measured = window[-1]["q"]
            residual = [a - b for a, b in zip(measured, target_q)]
            residual_max = max(abs(v) for v in residual)
            return {
                "schema_version": 2,
                "stable": True,
                "criterion": "self_stillness",
                "measured_q_rad": measured,
                "target_q_rad": target_q,
                "residual_rad": residual,
                "residual_max_rad": residual_max,
                "residual_within_reference": residual_max <= config.max_error_rad,
                "command_delivered": True,
                "command_settle_s": max(0.0, command_delivered_at - started),
                "window_started_monotonic": window[0]["timestamp"],
                "window_ended_monotonic": window[-1]["timestamp"],
                "sample_count": n,
                "max_velocity_rad_s": max(mean_dq),
                "peak_raw_velocity_rad_s": max(abs(v) for row in window for v in row["dq"]),
                "max_gyro_rad_s": mean_gyro,
                "imu_available": gyro_seen,
                "joint_range_rad": ranges,
                "max_state_age_s": max(row["age_s"] for row in window),
                "total_wait_s": now - started,
                "thresholds": asdict(config),
            }
        sleep(poll_s)
    reasons = []
    if last_diag is None:
        reasons.append("no fresh lowstate window" if command_delivered_at else "command never delivered")
    else:
        if max(last_diag["joint_range_rad"]) > config.max_range_rad:
            reasons.append(f"drift {max(last_diag['joint_range_rad']):.4f}>{config.max_range_rad} rad")
        if max(last_diag["mean_velocity_rad_s"]) > config.max_velocity_rad_s:
            reasons.append(
                f"velocity {max(last_diag['mean_velocity_rad_s']):.3f}>{config.max_velocity_rad_s} rad/s"
            )
        g = last_diag["mean_gyro_rad_s"]
        if g is not None and g > config.max_gyro_rad_s:
            reasons.append(f"gyro {g:.3f}>{config.max_gyro_rad_s} rad/s")
    raise TimeoutError(
        f"arm did not become still within {config.timeout_s:.2f}s ({'; '.join(reasons) or 'unknown'})"
    )
