"""Shared process for the per-arm manipulator Python runtime."""

from __future__ import annotations

from typing import Iterable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from manipulator_actions.action_server import ManipulatorActionServer
from manipulator_actions.behavior_tree_runtime_server import (
    BehaviorTreeRuntimeServer,
)
from manipulator_actions.pose_stamped_control import (
    PoseStampedEndEffectorControl,
)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)

    config_node = Node("manipulator_runtime_host")
    config_node.declare_parameter("enable_pose_stamped_control", True)
    config_node.declare_parameter("enable_action_servers", True)
    config_node.declare_parameter("runtime_executor_threads", 4)
    enable_pose = bool(
        config_node.get_parameter("enable_pose_stamped_control").value
    )
    enable_actions = bool(
        config_node.get_parameter("enable_action_servers").value
    )
    executor_threads = max(
        int(config_node.get_parameter("runtime_executor_threads").value), 2
    )
    config_node.destroy_node()

    nodes = []
    if enable_pose:
        nodes.append(PoseStampedEndEffectorControl())
    if enable_actions:
        nodes.extend(
            [
                ManipulatorActionServer(),
                BehaviorTreeRuntimeServer(),
            ]
        )

    executor = MultiThreadedExecutor(num_threads=executor_threads)
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
