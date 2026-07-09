import pytest

from manipulator_actions.behavior_tree_specs import (
    BACKEND_BEHAVIOR_TREE_CPP,
    DETECT_OBJECT,
    GRASP_OBJECT,
    MOVE_RELATIVE,
    PLACE_OBJECT,
    SEQUENCE,
    WAIT,
    BehaviorTreeValidationError,
    behavior_tree_cpp_xml,
    normalize_tree,
)


def test_normalize_tree_accepts_sequence_root():
    spec = normalize_tree(
        {
            "name": "sample",
            "root": {
                "sequence": {
                    "children": [
                        {"move_relative": {"name": "lift", "up": 0.05}},
                        {"wait": {"seconds": 0.5}},
                    ]
                }
            },
        }
    )

    assert spec.name == "sample"
    assert spec.root.kind == SEQUENCE
    assert spec.root.children[0].kind == MOVE_RELATIVE
    assert spec.root.children[0].name == "lift"
    assert spec.root.children[1].kind == WAIT


def test_normalize_tree_accepts_existing_sequence_yaml():
    spec = normalize_tree(
        {"steps": [{"move_relative": {"up": 0.05}}, {"wait": {"seconds": 0.1}}]},
        default_name="from_sequence",
    )

    assert spec.name == "from_sequence"
    assert spec.root.kind == SEQUENCE
    assert [child.kind for child in spec.root.children] == [MOVE_RELATIVE, WAIT]


def test_normalize_tree_rejects_unknown_backend():
    with pytest.raises(BehaviorTreeValidationError, match="Unsupported"):
        normalize_tree({"backend": "nope", "root": {"wait": {"seconds": 0.1}}})


def test_normalize_tree_requires_composite_children():
    with pytest.raises(BehaviorTreeValidationError, match="children"):
        normalize_tree({"root": {"sequence": {"children": []}}})


def test_bt_cpp_xml_export_preserves_future_backend_shape():
    spec = normalize_tree(
        {
            "name": "future_tree",
            "backend": BACKEND_BEHAVIOR_TREE_CPP,
            "root": {
                "sequence": {
                    "children": [
                        {"move_relative": {"up": 0.05}},
                    ]
                }
            },
        }
    )

    xml = behavior_tree_cpp_xml(spec)

    assert 'BehaviorTree ID="future_tree"' in xml
    assert "MoveRelative" in xml
    assert 'up="0.05"' in xml


def test_normalize_tree_accepts_modular_pick_place_actions():
    spec = normalize_tree(
        {
            "name": "modular_pick_place",
            "root": {
                "sequence": {
                    "children": [
                        {
                            "detect_object": {
                                "name": "detect_red",
                                "query": "red_cube",
                                "kind": "cube",
                            }
                        },
                        {
                            "grasp_object": {
                                "name": "grasp_red",
                                "object_id": "red_cube",
                            }
                        },
                        {
                            "place_object": {
                                "name": "place_red",
                                "target_id": "blue_place_pad",
                            }
                        },
                    ]
                }
            },
        }
    )

    assert spec.root.children[0].kind == DETECT_OBJECT
    assert spec.root.children[1].kind == GRASP_OBJECT
    assert spec.root.children[2].kind == PLACE_OBJECT
