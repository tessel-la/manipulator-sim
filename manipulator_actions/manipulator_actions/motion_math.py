"""Small, testable helpers for Cartesian Servo targets."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Tuple


DIRECTION_AXES = {
    "forward": (1.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
}


@dataclass(frozen=True)
class PoseTarget:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class MotionLimits:
    max_linear_speed: float = 0.15
    max_angular_speed: float = 0.5
    linear_gain: float = 1.5
    angular_gain: float = 1.5


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return base-frame yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def direction_offsets(values: Mapping[str, float]) -> Tuple[float, float, float]:
    """Convert named direction distances into base-frame x/y/z offsets."""
    dx = dy = dz = 0.0
    for name, amount in values.items():
        if name not in DIRECTION_AXES:
            raise ValueError(f"Unsupported direction '{name}'")
        ax, ay, az = DIRECTION_AXES[name]
        dx += ax * float(amount)
        dy += ay * float(amount)
        dz += az * float(amount)
    return dx, dy, dz


def relative_target(current: PoseTarget, dx: float, dy: float, dz: float, dyaw: float) -> PoseTarget:
    return PoseTarget(
        x=current.x + dx,
        y=current.y + dy,
        z=current.z + dz,
        yaw=wrap_angle(current.yaw + dyaw),
    )


def target_error(current: PoseTarget, target: PoseTarget) -> Tuple[float, float, float, float, float]:
    ex = target.x - current.x
    ey = target.y - current.y
    ez = target.z - current.z
    linear_distance = math.sqrt(ex * ex + ey * ey + ez * ez)
    yaw_error = wrap_angle(target.yaw - current.yaw)
    return ex, ey, ez, linear_distance, yaw_error


def servo_command(
    current: PoseTarget,
    target: PoseTarget,
    limits: MotionLimits,
    position_tolerance: float,
    yaw_tolerance: float,
) -> Tuple[float, float, float, float, float, float]:
    """Return vx, vy, vz, wz, remaining distance, remaining yaw."""
    ex, ey, ez, distance, yaw_error = target_error(current, target)

    if distance <= position_tolerance:
        vx = vy = vz = 0.0
    else:
        speed = clamp(distance * limits.linear_gain, 0.0, limits.max_linear_speed)
        vx = speed * ex / distance
        vy = speed * ey / distance
        vz = speed * ez / distance

    if abs(yaw_error) <= yaw_tolerance:
        wz = 0.0
    else:
        wz = clamp(
            yaw_error * limits.angular_gain,
            -limits.max_angular_speed,
            limits.max_angular_speed,
        )

    return vx, vy, vz, wz, distance, abs(yaw_error)
