import pytest

from manipulator_actions.simulated_scene import (
    attached_position,
    attachment_offset,
    placement_position,
)


def test_attachment_follows_end_effector_without_changing_grasp_offset():
    offset = attachment_offset(
        object_position=(0.45, -0.12, 0.07),
        end_effector_position=(0.45, -0.12, 0.20),
    )

    assert offset == pytest.approx((0.0, 0.0, -0.13))
    assert attached_position(
        end_effector_position=(0.45, 0.22, 0.296),
        offset=offset,
    ) == pytest.approx((0.45, 0.22, 0.166))


def test_placement_centres_cube_on_top_of_pad():
    assert placement_position(
        target_position=(0.45, 0.22, 0.046),
        object_size=0.06,
        target_height=0.012,
    ) == pytest.approx((0.45, 0.22, 0.082))
