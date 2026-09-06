import math

import pytest

from calibration_replay.models import Plan, PlanNode, route_for_plan, validate_plan
from calibration_replay.motion import (
    QUINTIC_ACCELERATION_PEAK,
    QUINTIC_VELOCITY_PEAK,
    interpolate_segment,
    segment_duration,
)


def make_plan():
    plan = Plan.create("test", "hand_eye_2D_head", "http://capture")
    plan.nodes = [
        PlanNode("home", "home", "home", [0.0] * 7),
        PlanNode("a", "a", "sample", [0.1] * 7),
        PlanNode("b", "b", "transit", [0.2] * 7),
        PlanNode("off", "off", "sample", [0.3] * 7, enabled=False),
    ]
    return plan


def test_route_goes_forward_then_straight_home_and_excludes_disabled():
    route = route_for_plan(make_plan())
    assert [(node.id, leg) for node, leg in route] == [
        ("home", "forward"),
        ("a", "forward"),
        ("b", "forward"),
        ("home", "reverse"),
    ]


def test_return_leg_gap_is_rejected_up_front():
    plan = make_plan()
    plan.motion.max_adjacent_delta_rad = 0.15
    errors = validate_plan(plan, require_home=True)
    assert any(err.startswith("return leg b->home home") for err in errors)
    assert not any(err.startswith("adjacent nodes") for err in errors)
    with pytest.raises(ValueError, match="return leg"):
        route_for_plan(plan)
    # 在末尾追加回程过渡点后即可通过
    plan.nodes.append(PlanNode("back", "back", "transit", [0.1] * 7))
    assert validate_plan(plan, require_home=True) == []
    assert [n.id for n, _ in route_for_plan(plan)] == ["home", "a", "b", "back", "home"]


def test_quintic_endpoints_and_duration_limits():
    start, end = [0.0] * 7, [0.8] * 7
    duration = segment_duration(
        start, end, vmax_rad_s=0.2, amax_rad_s2=0.4, min_duration_s=0.1
    )
    assert duration >= QUINTIC_VELOCITY_PEAK * 0.8 / 0.2
    assert duration >= math.sqrt(QUINTIC_ACCELERATION_PEAK * 0.8 / 0.4)
    frames = list(interpolate_segment(start, end, duration_s=duration, rate_hz=50))
    assert frames[0] == start
    assert frames[-1] == pytest.approx(end)
    dt = duration / (len(frames) - 1)
    peak_velocity = max(
        abs(b[0] - a[0]) / dt for a, b in zip(frames, frames[1:])
    )
    assert peak_velocity <= 0.2 * 1.001


def test_plan_validation_rejects_duplicates_shape_limits_and_delta():
    plan = make_plan()
    plan.nodes[1].id = "home"
    plan.nodes[2].q_rad = [2.0] * 7
    errors = validate_plan(plan, limits=[[-1.0, 1.0]] * 7, require_home=True)
    assert any("duplicate" in error for error in errors)
    assert any("outside" in error for error in errors)
    assert any("delta" in error for error in errors)


def test_2d_run_validation_requires_explicit_camera_serial():
    plan = make_plan()
    errors = validate_plan(
        plan,
        require_home=True,
        require_capture_ready=True,
    )
    assert any("camera_serial" in error for error in errors)
    plan.camera_serial = "CP0T263000BE"
    assert validate_plan(
        plan,
        require_home=True,
        require_capture_ready=True,
    ) == []


def test_mirror_plan_flips_roll_and_yaw_and_swaps_arm():
    from calibration_replay.models import mirror_plan, mirror_q

    plan = make_plan()
    plan.arm = "right"
    plan.nodes[1].q_rad = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    mirrored = mirror_plan(plan)
    assert mirrored.arm == "left" and mirrored.id != plan.id and mirrored.draft
    assert mirrored.nodes[1].q_rad == [0.1, -0.2, -0.3, 0.4, -0.5, 0.6, -0.7]
    assert mirrored.joint_names[0] == "left_shoulder_pitch_joint"
    # 镜像两次回到原值：左右对等
    assert mirror_q(mirror_q([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])) == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    back = mirror_plan(mirrored)
    assert back.arm == "right" and back.nodes[1].q_rad == pytest.approx(plan.nodes[1].q_rad)


def test_best_insert_index_picks_smallest_detour():
    from calibration_replay.models import Plan, PlanNode, best_insert_index

    def node(name, x, role="sample", enabled=True):
        return PlanNode.create(name, role, [x, 0, 0, 0, 0, 0, 0], enabled=enabled)

    plan = Plan.create("p", "hand_eye_3D", "http://x")
    plan.nodes = [node("home", 0.0, role="home"), node("a", 1.0), node("b", 2.0), node("c", 3.0)]

    # 1.5 belongs between a(1) and b(2) → index 2
    assert best_insert_index(plan, [1.5, 0, 0, 0, 0, 0, 0]) == 2
    # 0.5 belongs between home and a → index 1
    assert best_insert_index(plan, [0.5, 0, 0, 0, 0, 0, 0]) == 1
    # 4.0: gaps b→c and c→home tie on detour (2.0); tie-break picks the one whose
    # longer new segment is shorter → before c (segments 2,1) rather than the end (1,4)
    assert best_insert_index(plan, [4.0, 0, 0, 0, 0, 0, 0]) == 3
    # a point past the last node on another joint sits between b and c or on the return
    # leg with equal detour (0.4); again the shorter longest-segment wins → before c
    assert best_insert_index(plan, [3.0, 0.4, 0, 0, 0, 0, 0]) == 3
    # disabled node is not a gap endpoint but keeps its place
    plan.nodes.insert(2, node("off", 1.5, enabled=False))
    # gaps now: home→a, a→b (spanning the disabled row), b→c, c→home; 1.7 goes before b → index 3
    assert best_insert_index(plan, [1.7, 0, 0, 0, 0, 0, 0]) == 3
    # re-placing an existing node ignores itself
    plan.nodes = [node("home", 0.0, role="home"), node("a", 1.0), node("b", 2.0), node("late", 1.5)]
    late = plan.nodes[-1]
    assert best_insert_index(plan, late.q_rad, exclude_id=late.id) == 2
    # no home → append
    plan.nodes = [node("a", 1.0), node("b", 2.0)]
    assert best_insert_index(plan, [1.5, 0, 0, 0, 0, 0, 0]) == 2
