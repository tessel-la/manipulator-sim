import math

import pytest

from manipulator_actions.motion_math import (
    MotionLimits,
    PoseTarget,
    direction_offsets,
    relative_target,
    servo_command,
    wrap_angle,
)


def test_direction_offsets_use_base_frame_axes():
    dx, dy, dz = direction_offsets({"forward": 0.1, "left": 0.2, "down": 0.05})

    assert dx == pytest.approx(0.1)
    assert dy == pytest.approx(0.2)
    assert dz == pytest.approx(-0.05)


def test_direction_offsets_reject_unknown_direction():
    with pytest.raises(ValueError):
        direction_offsets({"diagonal": 0.1})


def test_relative_target_wraps_yaw_delta():
    target = relative_target(PoseTarget(0.1, 0.2, 0.3, math.pi - 0.1), 1.0, 2.0, 3.0, 0.3)

    assert target.x == pytest.approx(1.1)
    assert target.y == pytest.approx(2.2)
    assert target.z == pytest.approx(3.3)
    assert target.yaw == pytest.approx(wrap_angle(math.pi + 0.2))


def test_servo_command_clamps_speeds_and_stops_inside_tolerances():
    current = PoseTarget(0.0, 0.0, 0.0, 0.0)
    target = PoseTarget(1.0, 0.0, 0.0, 1.0)
    limits = MotionLimits(max_linear_speed=0.2, max_angular_speed=0.4, linear_gain=10.0, angular_gain=10.0)

    vx, vy, vz, wz, distance, yaw_error = servo_command(current, target, limits, 0.01, 0.03)

    assert vx == pytest.approx(0.2)
    assert vy == pytest.approx(0.0)
    assert vz == pytest.approx(0.0)
    assert wz == pytest.approx(0.4)
    assert distance == pytest.approx(1.0)
    assert yaw_error == pytest.approx(1.0)

    vx, vy, vz, wz, _, _ = servo_command(current, PoseTarget(0.001, 0.0, 0.0, 0.001), limits, 0.01, 0.03)
    assert (vx, vy, vz, wz) == pytest.approx((0.0, 0.0, 0.0, 0.0))
