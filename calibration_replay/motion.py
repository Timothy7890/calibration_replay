from __future__ import annotations

import math
from collections.abc import Iterator, Sequence

QUINTIC_VELOCITY_PEAK = 1.875
QUINTIC_ACCELERATION_PEAK = 10.0 / math.sqrt(3.0)


def segment_duration(
    start: Sequence[float],
    end: Sequence[float],
    *,
    vmax_rad_s: float,
    amax_rad_s2: float,
    min_duration_s: float,
) -> float:
    delta = max(abs(float(b) - float(a)) for a, b in zip(start, end))
    velocity_time = QUINTIC_VELOCITY_PEAK * delta / vmax_rad_s
    acceleration_time = math.sqrt(QUINTIC_ACCELERATION_PEAK * delta / amax_rad_s2)
    return max(float(min_duration_s), velocity_time, acceleration_time)


def quintic_scale(u: float) -> float:
    u = min(1.0, max(0.0, float(u)))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def interpolate_segment(
    start: Sequence[float],
    end: Sequence[float],
    *,
    duration_s: float,
    rate_hz: float = 50.0,
) -> Iterator[list[float]]:
    start_q = [float(v) for v in start]
    end_q = [float(v) for v in end]
    count = max(1, int(math.ceil(duration_s * rate_hz)))
    yield start_q
    for index in range(1, count + 1):
        scale = quintic_scale(index / count)
        yield [a + (b - a) * scale for a, b in zip(start_q, end_q)]
