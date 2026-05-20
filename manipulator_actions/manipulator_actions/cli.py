"""Command line client for manipulator action servers."""

from __future__ import annotations

import argparse
from typing import Iterable

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from manipulator_action_interfaces.action import MoveEndEffector, RunSequence
from manipulator_actions.motion_math import direction_offsets


def _add_common_move_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--timeout", type=float, default=0.0)
    parser.add_argument("--position-tolerance", type=float, default=0.0)
    parser.add_argument("--yaw-tolerance", type=float, default=0.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control the simulated Panda end effector.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    move_parser = subparsers.add_parser("move", help="Run a single Cartesian move")
    move_subparsers = move_parser.add_subparsers(dest="move_mode", required=True)

    absolute = move_subparsers.add_parser("absolute", help="Move to base-frame x/y/z/yaw")
    absolute.add_argument("--x", type=float, required=True)
    absolute.add_argument("--y", type=float, required=True)
    absolute.add_argument("--z", type=float, required=True)
    absolute.add_argument("--yaw", type=float, default=0.0)
    _add_common_move_args(absolute)

    relative = move_subparsers.add_parser("relative", help="Move by direction offsets")
    for name in ["forward", "back", "left", "right", "up", "down"]:
        relative.add_argument(f"--{name}", type=float, default=0.0)
    relative.add_argument("--yaw", type=float, default=0.0)
    _add_common_move_args(relative)

    sequence_parser = subparsers.add_parser("sequence", help="Run a named YAML sequence")
    sequence_subparsers = sequence_parser.add_subparsers(dest="sequence_command", required=True)
    run = sequence_subparsers.add_parser("run", help="Run a sequence by name")
    run.add_argument("name")

    return parser


class ManipulatorCli(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_cli")
        self._move_client = ActionClient(self, MoveEndEffector, "move_end_effector")
        self._sequence_client = ActionClient(self, RunSequence, "run_sequence")

    def run_move(self, args: argparse.Namespace) -> int:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_end_effector action server is not available")
            return 1

        goal = MoveEndEffector.Goal()
        goal.relative = args.move_mode == "relative"
        goal.yaw = args.yaw
        goal.timeout = args.timeout
        goal.position_tolerance = args.position_tolerance
        goal.yaw_tolerance = args.yaw_tolerance

        if args.move_mode == "absolute":
            goal.x = args.x
            goal.y = args.y
            goal.z = args.z
        else:
            direction_values = {
                name: getattr(args, name)
                for name in ["forward", "back", "left", "right", "up", "down"]
                if getattr(args, name) != 0.0
            }
            dx, dy, dz = direction_offsets(direction_values)
            goal.x = dx
            goal.y = dy
            goal.z = dz

        return self._send_move_goal(goal)

    def run_sequence(self, args: argparse.Namespace) -> int:
        if not self._sequence_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("run_sequence action server is not available")
            return 1

        goal = RunSequence.Goal()
        goal.name = args.name

        send_future = self._sequence_client.send_goal_async(goal, feedback_callback=self._sequence_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Sequence goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(result.message)
            return 0

        self.get_logger().error(result.message)
        return 1

    def _send_move_goal(self, goal: MoveEndEffector.Goal) -> int:
        send_future = self._move_client.send_goal_async(goal, feedback_callback=self._move_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(
                "Move complete: "
                f"x={result.final_x:.3f}, y={result.final_y:.3f}, "
                f"z={result.final_z:.3f}, yaw={result.final_yaw:.3f}"
            )
            return 0

        self.get_logger().error(result.message)
        return 1

    def _move_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"{feedback.state}: distance={feedback.remaining_distance:.3f} m, "
            f"yaw={feedback.remaining_yaw:.3f} rad"
        )

    def _sequence_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Step {feedback.current_step}: {feedback.current_action}")


def main(argv: Iterable[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    rclpy.init()
    node = ManipulatorCli()
    try:
        if args.command == "move":
            exit_code = node.run_move(args)
        elif args.command == "sequence" and args.sequence_command == "run":
            exit_code = node.run_sequence(args)
        else:
            raise RuntimeError("Unsupported command")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
