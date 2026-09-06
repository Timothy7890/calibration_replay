import pytest

from calibration_replay.models import StabilityConfig
from calibration_replay.stability import wait_for_stability


class FakeTime:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_stability_certificate_contains_continuous_window():
    fake = FakeTime()

    def read():
        return {
            "q": [0.001] * 7,
            "dq": [0.002] * 7,
            "timestamp": fake.clock(),
        }

    config = StabilityConfig(
        window_s=0.1,
        max_error_rad=0.01,
        max_velocity_rad_s=0.01,
        max_range_rad=0.005,
        freshness_s=0.05,
        timeout_s=1.0,
        command_settle_s=0.2,
    )
    certificate = wait_for_stability(
        read,
        [0.0] * 7,
        config,
        clock=fake.clock,
        sleep=fake.sleep,
        poll_s=0.02,
    )
    assert certificate["stable"] is True
    assert certificate["criterion"] == "self_stillness"
    assert certificate["sample_count"] >= 6
    assert certificate["residual_max_rad"] == 0.001
    assert certificate["residual_within_reference"] is True
    assert certificate["measured_q_rad"] == [0.001] * 7
    assert max(certificate["joint_range_rad"]) == 0.0
    assert certificate["thresholds"]["freshness_s"] == 0.05


def test_stillness_is_accepted_even_when_far_from_planned_target():
    """规划值只是参考：手臂停在离目标 0.5 rad 的地方也算到位，只标记偏差。"""
    fake = FakeTime()

    def read():
        return {"q": [0.5] * 7, "dq": [0.0] * 7, "timestamp": fake.clock(),
                "cmd_gap_rad": 0.0, "gyro_rad_s": [0.0, 0.0, 0.0]}

    config = StabilityConfig(window_s=0.1, max_error_rad=0.05, timeout_s=1.0)
    certificate = wait_for_stability(read, [0.0] * 7, config, clock=fake.clock, sleep=fake.sleep)
    assert certificate["stable"] is True
    assert certificate["residual_within_reference"] is False
    assert certificate["residual_max_rad"] == 0.5
    assert certificate["max_gyro_rad_s"] == 0.0
    assert certificate["imu_available"] is True


def test_waits_for_command_delivery_before_judging_stillness():
    """限速滑动还没送完（desired≠cmd）时，即便实测暂时不动也不能算到位。"""
    fake = FakeTime()
    q = [0.0] * 7

    def read():
        gap = 0.2 if fake.clock() < 0.3 else 0.0
        return {"q": list(q), "dq": [0.0] * 7, "timestamp": fake.clock(), "cmd_gap_rad": gap}

    config = StabilityConfig(window_s=0.1, timeout_s=2.0, command_settle_s=5.0)
    certificate = wait_for_stability(read, [0.0] * 7, config, clock=fake.clock, sleep=fake.sleep)
    assert certificate["command_settle_s"] >= 0.3
    assert certificate["window_started_monotonic"] >= 0.3


def test_moving_arm_or_moving_torso_times_out():
    fake = FakeTime()

    def moving_arm():
        return {"q": [fake.clock()] * 7, "dq": [1.0] * 7, "timestamp": fake.clock(), "cmd_gap_rad": 0.0}

    def swaying_torso():
        return {"q": [0.0] * 7, "dq": [0.0] * 7, "timestamp": fake.clock(),
                "cmd_gap_rad": 0.0, "gyro_rad_s": [0.5, 0.0, 0.0]}

    config = StabilityConfig(window_s=0.1, timeout_s=0.5, max_gyro_rad_s=0.1)
    for reader in (moving_arm, swaying_torso):
        fake.now = 0.0
        with pytest.raises(TimeoutError):
            wait_for_stability(reader, [0.0] * 7, config, clock=fake.clock, sleep=fake.sleep)


def test_quantized_velocity_noise_on_a_still_joint_is_not_motion():
    """真机现象：肩关节在重力载荷下 dq 上报 ±0.075 rad/s 交替抖动，位置却一格不动。"""
    fake = FakeTime()
    tick = {"n": 0}

    def read():
        tick["n"] += 1
        sign = 1.0 if tick["n"] % 2 else -1.0
        return {"q": [-0.993] + [0.0] * 6, "dq": [0.075 * sign] + [0.0] * 6,
                "timestamp": fake.clock(), "cmd_gap_rad": 0.0}

    config = StabilityConfig(window_s=0.3, timeout_s=2.0, max_velocity_rad_s=0.04)
    certificate = wait_for_stability(read, [-1.04] + [0.0] * 6, config, clock=fake.clock, sleep=fake.sleep)
    assert certificate["stable"] is True
    assert certificate["max_velocity_rad_s"] < 0.04
    assert certificate["peak_raw_velocity_rad_s"] == 0.075
