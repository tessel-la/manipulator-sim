"""ROS2 action server for reusable manipulator motions and YAML routines."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Iterable, List

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from manipulator_action_interfaces.action import MoveEndEffector, RunSequence
from manipulator_actions.controller import ServoMotionController
from manipulator_actions.motion_math import PoseTarget, direction_offsets
from manipulator_actions.sequences import (
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    WAIT,
    SequenceStep,
    SequenceValidationError,
    load_sequence_file,
    resolve_sequence_file,
    sequence_search_paths,
)


class ManipulatorActionServer(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_action_server")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_frame", "panda_link8")
        self.declare_parameter("twist_topic", "servo_node/delta_twist_cmds")
        self.declare_parameter("pause_servo_service", "servo_node/pause_servo")
        self.declare_parameter(
            "switch_command_type_service", "servo_node/switch_command_type"
        )
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("timeout", 10.0)
        self.declare_parameter("tf_timeout", 5.0)
        self.declare_parameter("sequence_directories", "")

        self._controller = ServoMotionController(
            node=self,
            base_frame=self.get_parameter("base_frame").value,
            ee_frame=self.get_parameter("ee_frame").value,
            twist_topic=self.get_parameter("twist_topic").value,
            pause_servo_service=self.get_parameter("pause_servo_service").value,
            switch_command_type_service=self.get_parameter(
                "switch_command_type_service"
            ).value,
            tf_timeout_sec=float(self.get_parameter("tf_timeout").value),
        )

        self._move_server = ActionServer(
            self,
            MoveEndEffector,
            "move_end_effector",
            self._execute_move,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._sequence_server = ActionServer(
            self,
            RunSequence,
            "run_sequence",
            self._execute_sequence,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info("Manipulator action server ready")

    def destroy_node(self) -> bool:
        self._controller.destroy()
        return super().destroy_node()

    def _cancel_callback(self, _cancel_request):
        return CancelResponse.ACCEPT

    @property
    def _default_position_tolerance(self) -> float:
        return float(self.get_parameter("position_tolerance").value)

    @property
    def _default_yaw_tolerance(self) -> float:
        return float(self.get_parameter("yaw_tolerance").value)

    @property
    def _default_timeout(self) -> float:
        return float(self.get_parameter("timeout").value)

    def _execute_move(self, goal_handle):
        goal = goal_handle.request
        position_tolerance = (
            goal.position_tolerance
            if goal.position_tolerance > 0.0
            else self._default_position_tolerance
        )
        yaw_tolerance = (
            goal.yaw_tolerance
            if goal.yaw_tolerance > 0.0
            else self._default_yaw_tolerance
        )
        timeout = goal.timeout if goal.timeout > 0.0 else self._default_timeout

        def publish_feedback(distance: float, yaw_error: float, state: str) -> None:
            feedback = MoveEndEffector.Feedback()
            feedback.remaining_distance = distance
            feedback.remaining_yaw = yaw_error
            feedback.state = state
            goal_handle.publish_feedback(feedback)

        if not goal.relative:
            success, message, final_pose = self._controller.move_absolute(
                target=PoseTarget(goal.x, goal.y, goal.z, goal.yaw),
                position_tolerance=position_tolerance,
                yaw_tolerance=yaw_tolerance,
                timeout_sec=timeout,
                feedback=publish_feedback,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )
        else:
            success, message, final_pose = self._controller.move_relative(
                dx=goal.x,
                dy=goal.y,
                dz=goal.z,
                dyaw=goal.yaw,
                position_tolerance=position_tolerance,
                yaw_tolerance=yaw_tolerance,
                timeout_sec=timeout,
                feedback=publish_feedback,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )

        result = self._move_result(success, message, final_pose)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_sequence(self, goal_handle):
        name = goal_handle.request.name
        result = RunSequence.Result()
        completed_steps = 0

        try:
            sequence_file = resolve_sequence_file(name, self._sequence_paths())
            steps = load_sequence_file(sequence_file)
        except (FileNotFoundError, SequenceValidationError) as exc:
            result.success = False
            result.message = str(exc)
            result.completed_steps = 0
            goal_handle.abort()
            return result

        self.get_logger().info(f"Running sequence '{name}' with {len(steps)} step(s)")
        for index, step in enumerate(steps, start=1):
            if goal_handle.is_cancel_requested:
                self._controller.halt()
                result.success = False
                result.message = "Sequence canceled"
                result.completed_steps = completed_steps
                goal_handle.canceled()
                return result

            feedback = RunSequence.Feedback()
            feedback.current_step = index
            feedback.current_action = step.kind
            goal_handle.publish_feedback(feedback)

            success, message = self._run_step(
                step, cancel_requested=lambda: goal_handle.is_cancel_requested
            )
            if not success:
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = "Sequence canceled"
                    result.completed_steps = completed_steps
                    goal_handle.canceled()
                    return result
                result.success = False
                result.message = f"Step {index} ({step.kind}) failed: {message}"
                result.completed_steps = completed_steps
                goal_handle.abort()
                return result
            completed_steps += 1

        result.success = True
        result.message = f"Sequence '{name}' completed"
        result.completed_steps = completed_steps
        goal_handle.succeed()
        return result

    def _run_step(self, step: SequenceStep, cancel_requested) -> tuple[bool, str]:
        if step.kind == WAIT:
            deadline = time.monotonic() + step.params["seconds"]
            while time.monotonic() < deadline:
                if cancel_requested():
                    return False, "Sequence canceled"
                time.sleep(0.05)
            return True, "Wait completed"

        if step.kind == MOVE_ABSOLUTE:
            success, message, _ = self._controller.move_absolute(
                target=PoseTarget(
                    step.params["x"],
                    step.params["y"],
                    step.params["z"],
                    step.params["yaw"],
                ),
                position_tolerance=self._default_position_tolerance,
                yaw_tolerance=self._default_yaw_tolerance,
                timeout_sec=self._default_timeout,
                cancel_requested=cancel_requested,
            )
            return success, message

        if step.kind == MOVE_RELATIVE:
            directions = {
                name: amount
                for name, amount in step.params.items()
                if name in {"forward", "back", "left", "right", "up", "down"}
            }
            dx, dy, dz = direction_offsets(directions)
            success, message, _ = self._controller.move_relative(
                dx=dx,
                dy=dy,
                dz=dz,
                dyaw=step.params.get("yaw", 0.0),
                position_tolerance=self._default_position_tolerance,
                yaw_tolerance=self._default_yaw_tolerance,
                timeout_sec=self._default_timeout,
                cancel_requested=cancel_requested,
            )
            return success, message

        return False, f"Unsupported step kind '{step.kind}'"

    def _sequence_paths(self) -> List[Path]:
        package_share = get_package_share_directory("manipulator_actions")
        paths: List[Path] = sequence_search_paths(package_share)
        configured = str(self.get_parameter("sequence_directories").value)
        paths.extend(Path(path) for path in configured.split(":") if path)
        return paths

    @staticmethod
    def _move_result(success: bool, message: str, final_pose: PoseTarget):
        result = MoveEndEffector.Result()
        result.success = success
        result.message = message
        result.final_x = final_pose.x
        result.final_y = final_pose.y
        result.final_z = final_pose.z
        result.final_yaw = final_pose.yaw
        return result


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManipulatorActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
