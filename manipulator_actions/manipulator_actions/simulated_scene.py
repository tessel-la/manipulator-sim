"""Small geometry helpers for camera-visible simulated pick and place."""

from __future__ import annotations

from typing import Iterable, Tuple


Vector3 = Tuple[float, float, float]


def vector3(values: Iterable[float]) -> Vector3:
    x, y, z = values
    return float(x), float(y), float(z)


def attachment_offset(
    object_position: Iterable[float],
    end_effector_position: Iterable[float],
) -> Vector3:
    object_x, object_y, object_z = vector3(object_position)
    ee_x, ee_y, ee_z = vector3(end_effector_position)
    return object_x - ee_x, object_y - ee_y, object_z - ee_z


def attached_position(
    end_effector_position: Iterable[float],
    offset: Iterable[float],
) -> Vector3:
    ee_x, ee_y, ee_z = vector3(end_effector_position)
    offset_x, offset_y, offset_z = vector3(offset)
    return ee_x + offset_x, ee_y + offset_y, ee_z + offset_z


def placement_position(
    target_position: Iterable[float],
    object_size: float,
    target_height: float,
) -> Vector3:
    """Put a cubic object's centre on top of the target fixture."""
    target_x, target_y, target_z = vector3(target_position)
    return (
        target_x,
        target_y,
        target_z + max(float(target_height), 0.0) * 0.5
        + max(float(object_size), 0.0) * 0.5,
    )
