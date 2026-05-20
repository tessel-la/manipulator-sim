import pytest

from manipulator_actions.sequences import (
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    WAIT,
    SequenceValidationError,
    normalize_sequence,
)


def test_normalize_sequence_accepts_steps_mapping():
    steps = normalize_sequence(
        {
            "steps": [
                {"move_absolute": {"x": 0.4, "y": 0.0, "z": 0.3, "yaw": 0.2}},
                {"wait": {"seconds": 0.5}},
                {"move_relative": {"up": 0.05, "forward": 0.1}},
            ]
        }
    )

    assert [step.kind for step in steps] == [MOVE_ABSOLUTE, WAIT, MOVE_RELATIVE]
    assert steps[0].params["x"] == pytest.approx(0.4)
    assert steps[1].params["seconds"] == pytest.approx(0.5)
    assert steps[2].params["up"] == pytest.approx(0.05)


def test_normalize_sequence_accepts_top_level_list():
    steps = normalize_sequence([{"move_relative": {"yaw": 0.1}}])

    assert len(steps) == 1
    assert steps[0].kind == MOVE_RELATIVE


def test_normalize_sequence_requires_absolute_coordinates():
    with pytest.raises(SequenceValidationError, match="Missing required field 'z'"):
        normalize_sequence([{"move_absolute": {"x": 0.4, "y": 0.0}}])


def test_normalize_sequence_rejects_unknown_relative_fields():
    with pytest.raises(SequenceValidationError, match="Unsupported relative"):
        normalize_sequence([{"move_relative": {"diagonal": 0.1}}])


def test_normalize_sequence_rejects_negative_waits():
    with pytest.raises(SequenceValidationError, match="non-negative"):
        normalize_sequence([{"wait": {"seconds": -1.0}}])
