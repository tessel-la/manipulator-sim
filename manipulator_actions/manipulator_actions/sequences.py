"""YAML sequence loading and validation for manipulator actions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping

import yaml


MOVE_ABSOLUTE = "move_absolute"
MOVE_RELATIVE = "move_relative"
WAIT = "wait"
VALID_STEP_TYPES = {MOVE_ABSOLUTE, MOVE_RELATIVE, WAIT}


class SequenceValidationError(ValueError):
    """Raised when a sequence file is syntactically valid YAML but not runnable."""


@dataclass(frozen=True)
class SequenceStep:
    kind: str
    params: Dict[str, float]


def _require_number(params: Mapping[str, Any], name: str) -> float:
    if name not in params:
        raise SequenceValidationError(f"Missing required field '{name}'")
    value = params[name]
    if not isinstance(value, (int, float)):
        raise SequenceValidationError(f"Field '{name}' must be a number")
    return float(value)


def _optional_number(params: Mapping[str, Any], name: str, default: float = 0.0) -> float:
    value = params.get(name, default)
    if not isinstance(value, (int, float)):
        raise SequenceValidationError(f"Field '{name}' must be a number")
    return float(value)


def normalize_step(raw_step: Mapping[str, Any]) -> SequenceStep:
    if not isinstance(raw_step, Mapping):
        raise SequenceValidationError("Each sequence step must be a mapping")
    if len(raw_step) != 1:
        raise SequenceValidationError("Each step must contain exactly one action")

    kind, params = next(iter(raw_step.items()))
    if kind not in VALID_STEP_TYPES:
        raise SequenceValidationError(f"Unsupported sequence action '{kind}'")
    if params is None:
        params = {}
    if not isinstance(params, Mapping):
        raise SequenceValidationError(f"Parameters for '{kind}' must be a mapping")

    if kind == MOVE_ABSOLUTE:
        normalized = {
            "x": _require_number(params, "x"),
            "y": _require_number(params, "y"),
            "z": _require_number(params, "z"),
            "yaw": _optional_number(params, "yaw"),
        }
    elif kind == MOVE_RELATIVE:
        allowed = {"forward", "back", "left", "right", "up", "down", "yaw"}
        unknown = set(params) - allowed
        if unknown:
            raise SequenceValidationError(
                f"Unsupported relative field(s): {', '.join(sorted(unknown))}"
            )
        normalized = {name: _optional_number(params, name) for name in allowed if name in params}
        if not normalized:
            raise SequenceValidationError("Relative moves need at least one direction or yaw")
    else:
        normalized = {"seconds": _require_number(params, "seconds")}
        if normalized["seconds"] < 0.0:
            raise SequenceValidationError("Wait seconds must be non-negative")

    return SequenceStep(kind=kind, params=normalized)


def normalize_sequence(raw: Any) -> List[SequenceStep]:
    if isinstance(raw, Mapping):
        raw_steps = raw.get("steps")
    else:
        raw_steps = raw

    if not isinstance(raw_steps, list):
        raise SequenceValidationError("Sequence YAML must be a list or contain a 'steps' list")
    if not raw_steps:
        raise SequenceValidationError("Sequence must contain at least one step")

    return [normalize_step(step) for step in raw_steps]


def sequence_search_paths(package_share: str | Path) -> List[Path]:
    root = Path(package_share)
    return [root / "config" / "sequences"]


def resolve_sequence_file(name: str, search_paths: Iterable[Path]) -> Path:
    candidate_names = [name]
    if not name.endswith((".yaml", ".yml")):
        candidate_names.extend([f"{name}.yaml", f"{name}.yml"])

    for directory in search_paths:
        for candidate_name in candidate_names:
            candidate = Path(directory) / candidate_name
            if candidate.is_file():
                return candidate

    searched = ", ".join(str(path) for path in search_paths)
    raise FileNotFoundError(f"Sequence '{name}' was not found in: {searched}")


def load_sequence_file(path: str | Path) -> List[SequenceStep]:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    return normalize_sequence(raw)
