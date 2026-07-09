"""Backend-neutral behavior tree YAML loading and validation."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping
import xml.etree.ElementTree as ET

import yaml

from manipulator_actions.sequences import (
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    WAIT,
    SequenceStep,
    normalize_sequence,
)


SEQUENCE = "sequence"
SELECTOR = "selector"
PARALLEL = "parallel"
RUN_SEQUENCE = "run_sequence"
IMAGE_CAPTURE = "image_capture"
DETECT_OBJECT = "detect_object"
GRASP_OBJECT = "grasp_object"
PLACE_OBJECT = "place_object"
COMPOSITE_TYPES = {SEQUENCE, SELECTOR, PARALLEL}
LEAF_TYPES = {
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    WAIT,
    RUN_SEQUENCE,
    IMAGE_CAPTURE,
    DETECT_OBJECT,
    GRASP_OBJECT,
    PLACE_OBJECT,
}
VALID_NODE_TYPES = COMPOSITE_TYPES | LEAF_TYPES
BACKEND_PY_TREES = "py_trees"
BACKEND_BEHAVIOR_TREE_CPP = "behavior_tree_cpp"
VALID_BACKENDS = {BACKEND_PY_TREES, BACKEND_BEHAVIOR_TREE_CPP}


class BehaviorTreeValidationError(ValueError):
    """Raised when a behavior tree YAML file is not runnable."""


@dataclass(frozen=True)
class BehaviorNodeSpec:
    kind: str
    name: str
    params: Dict[str, Any] = field(default_factory=dict)
    children: List["BehaviorNodeSpec"] = field(default_factory=list)


@dataclass(frozen=True)
class BehaviorTreeSpec:
    name: str
    root: BehaviorNodeSpec
    backend: str = BACKEND_PY_TREES


def _require_mapping(value: Any, message: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise BehaviorTreeValidationError(message)
    return value


def _require_number(params: Mapping[str, Any], name: str) -> float:
    if name not in params:
        raise BehaviorTreeValidationError(f"Missing required field '{name}'")
    value = params[name]
    if not isinstance(value, (int, float)):
        raise BehaviorTreeValidationError(f"Field '{name}' must be a number")
    return float(value)


def _optional_number(
    params: Mapping[str, Any],
    name: str,
    default: float = 0.0,
) -> float:
    value = params.get(name, default)
    if not isinstance(value, (int, float)):
        raise BehaviorTreeValidationError(f"Field '{name}' must be a number")
    return float(value)


def _optional_string(params: Mapping[str, Any], name: str, default: str = "") -> str:
    value = params.get(name, default)
    if not isinstance(value, str):
        raise BehaviorTreeValidationError(f"Field '{name}' must be a string")
    return value


def _node_name(kind: str, params: Mapping[str, Any], fallback: str) -> str:
    name = params.get("name", fallback)
    if not isinstance(name, str) or not name:
        raise BehaviorTreeValidationError("Node name must be a non-empty string")
    return name


def _normalize_leaf(kind: str, params: Mapping[str, Any]) -> Dict[str, Any]:
    if kind == MOVE_ABSOLUTE:
        return {
            "x": _require_number(params, "x"),
            "y": _require_number(params, "y"),
            "z": _require_number(params, "z"),
            "yaw": _optional_number(params, "yaw"),
            "timeout": _optional_number(params, "timeout"),
            "position_tolerance": _optional_number(params, "position_tolerance"),
            "yaw_tolerance": _optional_number(params, "yaw_tolerance"),
        }

    if kind == MOVE_RELATIVE:
        allowed = {
            "forward",
            "back",
            "left",
            "right",
            "up",
            "down",
            "yaw",
            "timeout",
            "position_tolerance",
            "yaw_tolerance",
        }
        unknown = set(params) - allowed - {"name"}
        if unknown:
            raise BehaviorTreeValidationError(
                f"Unsupported relative field(s): {', '.join(sorted(unknown))}"
            )
        normalized = {
            name: _optional_number(params, name) for name in allowed if name in params
        }
        motion_fields = set(normalized) - {
            "timeout",
            "position_tolerance",
            "yaw_tolerance",
        }
        if not motion_fields:
            raise BehaviorTreeValidationError(
                "Relative moves need at least one direction or yaw"
            )
        return normalized

    if kind == WAIT:
        seconds = _require_number(params, "seconds")
        if seconds < 0.0:
            raise BehaviorTreeValidationError("Wait seconds must be non-negative")
        return {"seconds": seconds}

    if kind == IMAGE_CAPTURE:
        topic = params.get("topic", "camera/image_raw")
        if not isinstance(topic, str) or not topic:
            raise BehaviorTreeValidationError("image_capture topic must be a non-empty string")
        timeout = _optional_number(params, "timeout", 5.0)
        if timeout < 0.0:
            raise BehaviorTreeValidationError("image_capture timeout must be non-negative")
        return {"topic": topic, "timeout": timeout}

    if kind == DETECT_OBJECT:
        query = _optional_string(params, "query")
        object_kind = _optional_string(params, "kind")
        target_frame = _optional_string(params, "target_frame")
        timeout = _optional_number(params, "timeout", 5.0)
        if not query and not object_kind:
            raise BehaviorTreeValidationError("detect_object needs query or kind")
        if timeout < 0.0:
            raise BehaviorTreeValidationError("detect_object timeout must be non-negative")
        return {
            "query": query,
            "kind": object_kind,
            "target_frame": target_frame,
            "timeout": timeout,
        }

    if kind == GRASP_OBJECT:
        object_id = _optional_string(params, "object_id")
        if not object_id:
            raise BehaviorTreeValidationError("grasp_object needs object_id")
        return {
            "object_id": object_id,
            "hover_height": _optional_number(params, "hover_height"),
            "approach_height": _optional_number(params, "approach_height"),
            "lift_height": _optional_number(params, "lift_height"),
            "timeout": _optional_number(params, "timeout"),
        }

    if kind == PLACE_OBJECT:
        target_id = _optional_string(params, "target_id")
        if not target_id:
            raise BehaviorTreeValidationError("place_object needs target_id")
        return {
            "target_id": target_id,
            "object_id": _optional_string(params, "object_id"),
            "hover_height": _optional_number(params, "hover_height"),
            "release_height": _optional_number(params, "release_height"),
            "lift_height": _optional_number(params, "lift_height"),
            "timeout": _optional_number(params, "timeout"),
        }

    name = params.get("name")
    if not isinstance(name, str) or not name:
        raise BehaviorTreeValidationError("run_sequence requires a non-empty name")
    return {"name": name}


def normalize_node(
    raw_node: Mapping[str, Any], fallback_name: str = "node"
) -> BehaviorNodeSpec:
    raw_node = _require_mapping(raw_node, "Each behavior tree node must be a mapping")
    if len(raw_node) != 1:
        raise BehaviorTreeValidationError(
            "Each tree node must contain exactly one kind"
        )

    kind, raw_params = next(iter(raw_node.items()))
    if kind not in VALID_NODE_TYPES:
        raise BehaviorTreeValidationError(f"Unsupported behavior tree node '{kind}'")
    params = (
        {}
        if raw_params is None
        else _require_mapping(
            raw_params,
            f"Parameters for '{kind}' must be a mapping",
        )
    )
    name = _node_name(kind, params, fallback_name)

    if kind in COMPOSITE_TYPES:
        raw_children = params.get("children")
        if not isinstance(raw_children, list) or not raw_children:
            raise BehaviorTreeValidationError(
                f"Composite node '{name}' needs a non-empty children list"
            )
        children = [
            normalize_node(child, fallback_name=f"{name}_{index}")
            for index, child in enumerate(raw_children, start=1)
        ]
        composite_params = {
            key: value
            for key, value in params.items()
            if key not in {"children", "name"}
        }
        return BehaviorNodeSpec(
            kind=kind, name=name, params=composite_params, children=children
        )

    return BehaviorNodeSpec(
        kind=kind,
        name=name,
        params=_normalize_leaf(kind, params),
    )


def tree_from_sequence_steps(name: str, steps: List[SequenceStep]) -> BehaviorTreeSpec:
    children = [
        BehaviorNodeSpec(
            kind=step.kind, name=f"{index}_{step.kind}", params=step.params
        )
        for index, step in enumerate(steps, start=1)
    ]
    root = BehaviorNodeSpec(
        kind=SEQUENCE,
        name=name,
        params={"memory": True},
        children=children,
    )
    return BehaviorTreeSpec(name=name, root=root)


def normalize_tree(raw: Any, default_name: str = "behavior_tree") -> BehaviorTreeSpec:
    if isinstance(raw, list) or (
        isinstance(raw, Mapping) and "steps" in raw and "root" not in raw
    ):
        return tree_from_sequence_steps(default_name, normalize_sequence(raw))

    raw_tree = _require_mapping(raw, "Behavior tree YAML must be a mapping")
    name = raw_tree.get("name", default_name)
    if not isinstance(name, str) or not name:
        raise BehaviorTreeValidationError("Tree name must be a non-empty string")

    backend = raw_tree.get("backend", BACKEND_PY_TREES)
    if backend not in VALID_BACKENDS:
        raise BehaviorTreeValidationError(
            f"Unsupported behavior tree backend '{backend}'"
        )

    if "root" not in raw_tree:
        raise BehaviorTreeValidationError("Behavior tree YAML needs a root node")
    return BehaviorTreeSpec(
        name=name,
        backend=backend,
        root=normalize_node(raw_tree["root"], fallback_name=name),
    )


def behavior_tree_search_paths(package_share: str | Path) -> List[Path]:
    root = Path(package_share)
    return [root / "config" / "trees", root / "config" / "sequences"]


def resolve_behavior_tree_file(name: str, search_paths: Iterable[Path]) -> Path:
    candidate_names = [name]
    if not name.endswith((".yaml", ".yml")):
        candidate_names.extend([f"{name}.yaml", f"{name}.yml"])

    for directory in search_paths:
        for candidate_name in candidate_names:
            candidate = Path(directory) / candidate_name
            if candidate.is_file():
                return candidate

    searched = ", ".join(str(path) for path in search_paths)
    raise FileNotFoundError(f"Behavior tree '{name}' was not found in: {searched}")


def load_behavior_tree_file(path: str | Path) -> BehaviorTreeSpec:
    tree_path = Path(path)
    with tree_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    return normalize_tree(raw, default_name=tree_path.stem)


def behavior_tree_cpp_xml(spec: BehaviorTreeSpec) -> str:
    """Export a simple BehaviorTree.CPP-style XML skeleton for future runners."""
    root = ET.Element("root", {"BTCPP_format": "4"})
    tree = ET.SubElement(root, "BehaviorTree", {"ID": spec.name})
    tree.append(_node_to_bt_cpp(spec.root))
    return ET.tostring(root, encoding="unicode")


def _node_to_bt_cpp(spec: BehaviorNodeSpec) -> ET.Element:
    tag_by_kind = {
        SEQUENCE: "Sequence",
        SELECTOR: "Fallback",
        PARALLEL: "Parallel",
        MOVE_ABSOLUTE: "MoveAbsolute",
        MOVE_RELATIVE: "MoveRelative",
        WAIT: "Wait",
        RUN_SEQUENCE: "RunSequence",
        IMAGE_CAPTURE: "ImageCapture",
        DETECT_OBJECT: "DetectObject",
        GRASP_OBJECT: "GraspObject",
        PLACE_OBJECT: "PlaceObject",
    }
    element = ET.Element(tag_by_kind[spec.kind], {"name": spec.name})
    for key, value in spec.params.items():
        element.set(key, str(value))
    for child in spec.children:
        element.append(_node_to_bt_cpp(child))
    return element
