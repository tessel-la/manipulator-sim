"""Runtime server for editing and executing manipulator behavior trees."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Dict, Iterable, Optional

from ament_index_python.packages import get_package_share_directory
import py_trees
import rclpy
from rclpy.node import Node
import yaml
from std_msgs.msg import String

from manipulator_actions.behavior_tree_specs import (
    BACKEND_BEHAVIOR_TREE_CPP,
    BACKEND_PY_TREES,
    IMAGE_CAPTURE,
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    PARALLEL,
    RUN_SEQUENCE,
    SELECTOR,
    SEQUENCE,
    WAIT,
    BehaviorTreeSpec,
    behavior_tree_cpp_xml,
    behavior_tree_search_paths,
    load_behavior_tree_file,
    normalize_tree,
)
from manipulator_actions.py_trees_runner import build_py_tree


NODE_TYPES = [
    {"id": SEQUENCE, "label": "Sequence", "category": "control", "minChildren": 1},
    {"id": SELECTOR, "label": "Selector", "category": "control", "minChildren": 1},
    {"id": PARALLEL, "label": "Parallel", "category": "control", "minChildren": 1},
    {
        "id": MOVE_ABSOLUTE,
        "label": "Move Absolute",
        "category": "action",
        "params": [
            {"name": "x", "type": "number", "required": True, "default": 0.45},
            {"name": "y", "type": "number", "required": True, "default": 0.0},
            {"name": "z", "type": "number", "required": True, "default": 0.35},
            {"name": "yaw", "type": "number", "default": 0.0},
        ],
    },
    {
        "id": MOVE_RELATIVE,
        "label": "Move Relative",
        "category": "action",
        "params": [
            {"name": "up", "type": "number", "default": 0.05},
            {"name": "forward", "type": "number", "default": 0.0},
            {"name": "yaw", "type": "number", "default": 0.0},
        ],
    },
    {
        "id": WAIT,
        "label": "Wait",
        "category": "action",
        "params": [{"name": "seconds", "type": "number", "required": True, "default": 0.5}],
        "maxChildren": 0,
    },
    {
        "id": RUN_SEQUENCE,
        "label": "Run Sequence",
        "category": "action",
        "params": [{"name": "name", "type": "string", "required": True, "default": "demo_pick"}],
        "maxChildren": 0,
    },
    {
        "id": IMAGE_CAPTURE,
        "label": "Image Capture",
        "category": "action",
        "params": [
            {"name": "topic", "type": "string", "required": True, "default": "camera/image_raw"},
            {"name": "timeout", "type": "number", "default": 5.0},
        ],
        "maxChildren": 0,
    },
]


class BehaviorTreeRuntimeServer(Node):
    def __init__(self) -> None:
        super().__init__("behavior_tree_runtime_server")
        self.declare_parameter("engine", BACKEND_PY_TREES)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("default_timeout", 60.0)

        self.engine = str(self.get_parameter("engine").value)
        self.tick_hz = max(float(self.get_parameter("tick_hz").value), 1.0)
        self.default_timeout = max(float(self.get_parameter("default_timeout").value), 0.0)

        self.capabilities_pub = self.create_publisher(String, "behavior_tree/runtime/capabilities", 10)
        self.tree_catalog_pub = self.create_publisher(String, "behavior_tree/runtime/trees", 10)
        self.node_catalog_pub = self.create_publisher(String, "behavior_tree/runtime/nodes", 10)
        self.status_pub = self.create_publisher(String, "behavior_tree/runtime/status", 10)
        self.tree_pub = self.create_publisher(String, "behavior_tree/runtime/tree", 10)

        self.create_subscription(String, "behavior_tree/runtime/spec", self._on_spec, 10)
        self.create_subscription(String, "behavior_tree/runtime/command", self._on_command, 10)

        self.specs: Dict[str, tuple[BehaviorTreeSpec, str]] = self._load_available_specs()
        self.active_spec: Optional[BehaviorTreeSpec] = None
        self.active_root = None
        self.active_tree: Optional[py_trees.trees.BehaviourTree] = None
        self.deadline: Optional[float] = None
        self.is_running = False

        self.create_timer(1.0, self._publish_capabilities)
        self.create_timer(1.0 / self.tick_hz, self._tick_active_tree)
        self._publish_capabilities()
        self.get_logger().info("Behavior tree runtime server ready")

    def _load_available_specs(self) -> Dict[str, tuple[BehaviorTreeSpec, str]]:
        specs: Dict[str, tuple[BehaviorTreeSpec, str]] = {}
        package_share = get_package_share_directory("manipulator_actions")
        for directory in behavior_tree_search_paths(package_share):
            if not directory.is_dir():
                continue
            for path in sorted(Path(directory).glob("*.y*ml")):
                try:
                    spec = load_behavior_tree_file(path)
                    specs[spec.name] = (spec, path.read_text(encoding="utf-8"))
                except Exception as exc:
                    self.get_logger().warning(f"Skipping invalid behavior tree {path}: {exc}")
        return specs

    def _publish_json(self, publisher, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(message)

    def _tree_catalog(self):
        return [
            {
                "id": name,
                "name": spec.name,
                "engine": spec.backend,
                "format": "yaml",
                "spec": source,
            }
            for name, (spec, source) in sorted(self.specs.items())
        ]

    def _publish_capabilities(self) -> None:
        payload = {
            "engine": self.engine,
            "nodeTypes": NODE_TYPES,
            "trees": self._tree_catalog(),
            "constraints": [
                "Exactly one root node is required",
                "Composite nodes need at least one child",
                "Leaf action nodes cannot have children",
            ],
        }
        self._publish_json(self.capabilities_pub, payload)
        self._publish_json(self.tree_catalog_pub, {"trees": payload["trees"]})

    def _on_spec(self, message: String) -> None:
        try:
            raw = yaml.safe_load(message.data)
            spec = normalize_tree(raw)
            self.specs[spec.name] = (spec, message.data)
            self.active_spec = spec
            self._publish_capabilities()
            self.get_logger().info(f"Loaded behavior tree spec '{spec.name}' from UI")
        except Exception as exc:
            self.get_logger().error(f"Rejected behavior tree spec: {exc}")

    def _on_command(self, message: String) -> None:
        try:
            command = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid behavior tree command JSON: {exc}")
            return

        action = str(command.get("command", "")).lower()
        tree_id = command.get("treeId") or command.get("name")
        if action in {"load", "run", "load_and_run"}:
            if tree_id and str(tree_id) in self.specs:
                self.active_spec = self.specs[str(tree_id)][0]
            if self.active_spec is None:
                self.get_logger().error("No behavior tree spec selected")
                return
            self._build_active_tree()
            if action in {"run", "load_and_run"}:
                self.is_running = True
                self.deadline = (
                    time.monotonic() + self.default_timeout
                    if self.default_timeout > 0.0
                    else None
                )
                self.get_logger().info(f"Running behavior tree '{self.active_spec.name}'")
                self._publish_runtime_tree()
            return

        if action == "stop":
            self.is_running = False
            self.deadline = None
            self.get_logger().info("Stopped behavior tree runtime")
            self._publish_runtime_status()
            return

        self.get_logger().warning(f"Unknown behavior tree command '{action}'")

    def _build_active_tree(self) -> None:
        if self.active_spec is None:
            return
        self.active_root = build_py_tree(self, self.active_spec.root)
        self.active_tree = py_trees.trees.BehaviourTree(self.active_root)
        self._publish_runtime_tree()

    def _walk_runtime_nodes(self, behaviour, parent_id: Optional[str] = None, path: Optional[list[str]] = None):
        path = path or []
        name = str(behaviour.name)
        node_id = "/".join([*path, name])
        status = str(behaviour.status.name) if behaviour.status is not None else "IDLE"
        node = {
            "id": node_id,
            "name": name,
            "type": behaviour.__class__.__name__,
            "status": status,
            "treeId": self.active_spec.name if self.active_spec else "runtime",
            "path": node_id,
            "source": "py_trees",
        }
        if parent_id is not None:
            node["parentId"] = parent_id
        nodes = [node]
        for child in getattr(behaviour, "children", []) or []:
            nodes.extend(self._walk_runtime_nodes(child, node_id, [*path, name]))
        return nodes

    def _publish_runtime_tree(self) -> None:
        if self.active_spec is None or self.active_root is None:
            return
        nodes = self._walk_runtime_nodes(self.active_root)
        self._publish_json(
            self.node_catalog_pub,
            {
                "trees": [
                    {
                        "id": self.active_spec.name,
                        "name": self.active_spec.name,
                        "engine": self.active_spec.backend,
                        "nodes": nodes,
                    }
                ]
            },
        )
        message = String()
        message.data = behavior_tree_cpp_xml(self.active_spec)
        self.tree_pub.publish(message)

    def _publish_runtime_status(self) -> None:
        if self.active_root is None:
            return
        nodes = self._walk_runtime_nodes(self.active_root)
        self._publish_json(self.status_pub, {"nodes": {node["id"]: node["status"] for node in nodes}})
        self._publish_runtime_tree()

    def _tick_active_tree(self) -> None:
        if not self.is_running or self.active_tree is None or self.active_root is None:
            return
        self.active_tree.tick()
        self._publish_runtime_status()
        if self.active_root.status in {py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE}:
            self.is_running = False
            self.get_logger().info(
                f"Behavior tree '{self.active_spec.name if self.active_spec else 'runtime'}' "
                f"finished with {self.active_root.status.name}"
            )
        if self.deadline is not None and time.monotonic() > self.deadline:
            self.is_running = False
            self.get_logger().error("Behavior tree runtime timed out")


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BehaviorTreeRuntimeServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
