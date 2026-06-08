"""ROS2 py_trees runner for manipulator behavior tree specs."""

from __future__ import annotations

import argparse
from pathlib import Path
import time
from typing import Iterable, List, Optional

from ament_index_python.packages import get_package_share_directory
import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from manipulator_action_interfaces.action import MoveEndEffector, RunSequence
from manipulator_actions.behavior_tree_specs import (
    BACKEND_BEHAVIOR_TREE_CPP,
    BACKEND_PY_TREES,
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    PARALLEL,
    RUN_SEQUENCE,
    SELECTOR,
    SEQUENCE,
    WAIT,
    BehaviorNodeSpec,
    BehaviorTreeSpec,
    behavior_tree_cpp_xml,
    behavior_tree_search_paths,
    load_behavior_tree_file,
    resolve_behavior_tree_file,
)
from manipulator_actions.motion_math import direction_offsets


class TimedWait(py_trees.behaviour.Behaviour):
    def __init__(self, spec: BehaviorNodeSpec) -> None:
        super().__init__(name=spec.name)
        self._seconds = float(spec.params["seconds"])
        self._deadline: Optional[float] = None

    def initialise(self) -> None:
        self._deadline = time.monotonic() + self._seconds

    def update(self):
        if self._deadline is not None and time.monotonic() >= self._deadline:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class MoveEndEffectorAction(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, spec: BehaviorNodeSpec) -> None:
        super().__init__(name=spec.name)
        self._node = node
        self._spec = spec
        self._client = ActionClient(node, MoveEndEffector, "move_end_effector")
        self._server_deadline: Optional[float] = None
        self._send_future = None
        self._result_future = None
        self.feedback_message = "idle"

    def initialise(self) -> None:
        self._server_deadline = time.monotonic() + 10.0
        self._send_future = None
        self._result_future = None
        self.feedback_message = "waiting for move_end_effector"

    def update(self):
        if self._result_future is not None:
            if not self._result_future.done():
                return py_trees.common.Status.RUNNING
            result = self._result_future.result().result
            self.feedback_message = result.message
            return (
                py_trees.common.Status.SUCCESS
                if result.success
                else py_trees.common.Status.FAILURE
            )

        if self._send_future is not None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            goal_handle = self._send_future.result()
            if not goal_handle.accepted:
                self.feedback_message = "move goal rejected"
                return py_trees.common.Status.FAILURE
            self.feedback_message = "moving"
            self._result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self._client.wait_for_server(timeout_sec=0.0):
            self._send_future = self._client.send_goal_async(self._goal())
            self.feedback_message = "move goal sent"
            return py_trees.common.Status.RUNNING

        if (
            self._server_deadline is not None
            and time.monotonic() > self._server_deadline
        ):
            self.feedback_message = "move_end_effector action server is not available"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def _goal(self) -> MoveEndEffector.Goal:
        params = self._spec.params
        goal = MoveEndEffector.Goal()
        goal.relative = self._spec.kind == MOVE_RELATIVE
        goal.yaw = float(params.get("yaw", 0.0))
        goal.timeout = float(params.get("timeout", 0.0))
        goal.position_tolerance = float(params.get("position_tolerance", 0.0))
        goal.yaw_tolerance = float(params.get("yaw_tolerance", 0.0))

        if self._spec.kind == MOVE_ABSOLUTE:
            goal.x = float(params["x"])
            goal.y = float(params["y"])
            goal.z = float(params["z"])
            return goal

        directions = {
            name: float(params[name])
            for name in ["forward", "back", "left", "right", "up", "down"]
            if name in params
        }
        goal.x, goal.y, goal.z = direction_offsets(directions)
        return goal


class RunSequenceAction(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, spec: BehaviorNodeSpec) -> None:
        super().__init__(name=spec.name)
        self._node = node
        self._spec = spec
        self._client = ActionClient(node, RunSequence, "run_sequence")
        self._server_deadline: Optional[float] = None
        self._send_future = None
        self._result_future = None
        self.feedback_message = "idle"

    def initialise(self) -> None:
        self._server_deadline = time.monotonic() + 10.0
        self._send_future = None
        self._result_future = None
        self.feedback_message = "waiting for run_sequence"

    def update(self):
        if self._result_future is not None:
            if not self._result_future.done():
                return py_trees.common.Status.RUNNING
            result = self._result_future.result().result
            self.feedback_message = result.message
            return (
                py_trees.common.Status.SUCCESS
                if result.success
                else py_trees.common.Status.FAILURE
            )

        if self._send_future is not None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            goal_handle = self._send_future.result()
            if not goal_handle.accepted:
                self.feedback_message = "sequence goal rejected"
                return py_trees.common.Status.FAILURE
            self.feedback_message = "running sequence"
            self._result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self._client.wait_for_server(timeout_sec=0.0):
            goal = RunSequence.Goal()
            goal.name = str(self._spec.params["name"])
            self._send_future = self._client.send_goal_async(goal)
            self.feedback_message = "sequence goal sent"
            return py_trees.common.Status.RUNNING

        if (
            self._server_deadline is not None
            and time.monotonic() > self._server_deadline
        ):
            self.feedback_message = "run_sequence action server is not available"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


def build_py_tree(node: Node, spec: BehaviorNodeSpec):
    if spec.kind == SEQUENCE:
        composite = py_trees.composites.Sequence(
            name=spec.name,
            memory=bool(spec.params.get("memory", True)),
        )
    elif spec.kind == SELECTOR:
        composite = py_trees.composites.Selector(
            name=spec.name,
            memory=bool(spec.params.get("memory", True)),
        )
    elif spec.kind == PARALLEL:
        composite = py_trees.composites.Parallel(
            name=spec.name,
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=bool(spec.params.get("synchronise", False)),
            ),
        )
    elif spec.kind in {MOVE_ABSOLUTE, MOVE_RELATIVE}:
        return MoveEndEffectorAction(node, spec)
    elif spec.kind == RUN_SEQUENCE:
        return RunSequenceAction(node, spec)
    else:
        return TimedWait(spec)

    composite.add_children([build_py_tree(node, child) for child in spec.children])
    return composite


class PyTreesRunner(Node):
    def __init__(self, spec: BehaviorTreeSpec, tick_hz: float, timeout: float) -> None:
        super().__init__("manipulator_py_trees_runner")
        self._spec = spec
        self._tick_hz = max(tick_hz, 1.0)
        self._timeout = max(timeout, 0.0)
        self._root = build_py_tree(self, spec.root)
        self._tree = py_trees.trees.BehaviourTree(self._root)

    def run(self) -> int:
        self.get_logger().info(f"Running behavior tree '{self._spec.name}'")
        deadline = time.monotonic() + self._timeout if self._timeout > 0.0 else None
        period = 1.0 / self._tick_hz

        while rclpy.ok():
            self._tree.tick()
            rclpy.spin_once(self, timeout_sec=0.0)

            if self._root.status == py_trees.common.Status.SUCCESS:
                self.get_logger().info(f"Behavior tree '{self._spec.name}' succeeded")
                return 0
            if self._root.status == py_trees.common.Status.FAILURE:
                self.get_logger().error(f"Behavior tree '{self._spec.name}' failed")
                return 1
            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().error(f"Behavior tree '{self._spec.name}' timed out")
                return 1
            time.sleep(period)

        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run manipulator behavior trees.")
    parser.add_argument("name", help="Tree YAML name or path")
    parser.add_argument(
        "--backend",
        choices=[BACKEND_PY_TREES, BACKEND_BEHAVIOR_TREE_CPP],
        default=BACKEND_PY_TREES,
    )
    parser.add_argument("--tree-directory", action="append", default=[])
    parser.add_argument("--tick-hz", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--export-btcpp-xml",
        action="store_true",
        help="Print a BehaviorTree.CPP XML skeleton and exit",
    )
    return parser


def _tree_paths(extra_directories: Iterable[str]) -> List[Path]:
    package_share = get_package_share_directory("manipulator_actions")
    paths = behavior_tree_search_paths(package_share)
    paths.extend(Path(path) for path in extra_directories if path)
    return paths


def _load_spec(name: str, extra_directories: Iterable[str]) -> BehaviorTreeSpec:
    tree_path = Path(name)
    if tree_path.is_file():
        return load_behavior_tree_file(tree_path)
    return load_behavior_tree_file(
        resolve_behavior_tree_file(name, _tree_paths(extra_directories))
    )


def _application_args(argv: Iterable[str] | None) -> List[str]:
    if argv is None:
        return remove_ros_args()[1:]
    return remove_ros_args(args=list(argv))


def main(argv: Iterable[str] | None = None) -> None:
    args = build_parser().parse_args(_application_args(argv))
    spec = _load_spec(args.name, args.tree_directory)

    if args.export_btcpp_xml or args.backend == BACKEND_BEHAVIOR_TREE_CPP:
        print(behavior_tree_cpp_xml(spec))
        if args.export_btcpp_xml:
            return
        raise SystemExit("BehaviorTree.CPP execution is not implemented yet")

    rclpy.init()
    node = PyTreesRunner(spec=spec, tick_hz=args.tick_hz, timeout=args.timeout)
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
