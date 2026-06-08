"""Topic bridge for PoseStamped end-effector Servo control."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Iterable, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from manipulator_actions.controller import ServoMotionController
from manipulator_actions.motion_math import (
    MotionLimits,
    PoseTarget,
    yaw_from_quaternion,
)


@dataclass(frozen=True)
class PoseCommand:
    target: PoseTarget
    relative: bool
    source_topic: str


class PoseStampedEndEffectorControl(Node):
    """Move the end effector from PoseStamped absolute targets or offsets."""

    def __init__(self) -> None:
        super().__init__("pose_stamped_end_effector_control")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_frame", "panda_link8")
        self.declare_parameter("twist_topic", "servo_node/delta_twist_cmds")
        self.declare_parameter("pause_servo_service", "servo_node/pause_servo")
        self.declare_parameter(
            "switch_command_type_service", "servo_node/switch_command_type"
        )
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("relative", False)
        self.declare_parameter("absolute_pose_topic", "end_effector_pose_absolute")
        self.declare_parameter("relative_pose_topic", "end_effector_pose_relative")
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("timeout", 10.0)
        self.declare_parameter("tf_timeout", 5.0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("linear_gain", 1.5)
        self.declare_parameter("angular_gain", 1.5)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._default_relative = bool(self.get_parameter("relative").value)
        self._position_tolerance = float(self.get_parameter("position_tolerance").value)
        self._yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self._timeout = float(self.get_parameter("timeout").value)

        self._controller = ServoMotionController(
            node=self,
            base_frame=self._base_frame,
            ee_frame=self.get_parameter("ee_frame").value,
            twist_topic=self.get_parameter("twist_topic").value,
            pause_servo_service=self.get_parameter("pause_servo_service").value,
            switch_command_type_service=self.get_parameter(
                "switch_command_type_service"
            ).value,
            publish_hz=float(self.get_parameter("publish_hz").value),
            tf_timeout_sec=float(self.get_parameter("tf_timeout").value),
            limits=MotionLimits(
                max_linear_speed=float(self.get_parameter("max_linear_speed").value),
                max_angular_speed=float(self.get_parameter("max_angular_speed").value),
                linear_gain=float(self.get_parameter("linear_gain").value),
                angular_gain=float(self.get_parameter("angular_gain").value),
            ),
        )

        self._condition = threading.Condition()
        self._pending_command: Optional[PoseCommand] = None
        self._active_command: Optional[PoseCommand] = None
        self._stop_requested = False
        self._worker = threading.Thread(target=self._run_worker, daemon=True)

        self._subscription_topics = []
        self._subscriptions = []
        self._add_subscription(
            str(self.get_parameter("pose_topic").value),
            self._default_relative,
        )
        self._add_subscription(
            str(self.get_parameter("absolute_pose_topic").value),
            False,
        )
        self._add_subscription(
            str(self.get_parameter("relative_pose_topic").value),
            True,
        )

        if not self._subscription_topics:
            self.get_logger().warn("No PoseStamped control topics configured")
        else:
            topics = ", ".join(topic for topic, _ in self._subscription_topics)
            self.get_logger().info(
                f"Listening for PoseStamped end-effector targets on {topics}"
            )

        self._worker.start()

    def destroy_node(self) -> bool:
        with self._condition:
            self._stop_requested = True
            self._condition.notify_all()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        self._controller.halt()
        self._controller.destroy()
        return super().destroy_node()

    def _add_subscription(self, topic: str, relative: bool) -> None:
        topic = topic.strip()
        if not topic:
            return
        if any(
            existing_topic == topic for existing_topic, _ in self._subscription_topics
        ):
            self.get_logger().warn(f"Ignoring duplicate PoseStamped topic '{topic}'")
            return

        subscription = self.create_subscription(
            PoseStamped,
            topic,
            lambda message, topic=topic, relative=relative: self._store_command(
                message, topic, relative
            ),
            10,
            callback_group=self._callback_group,
        )
        self._subscriptions.append(subscription)
        self._subscription_topics.append((topic, relative))

    def _store_command(
        self, message: PoseStamped, source_topic: str, relative: bool
    ) -> None:
        try:
            target = (
                self._target_from_relative_pose(message)
                if relative
                else self._controller.target_from_pose_stamped(message)
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Ignoring PoseStamped on {source_topic}: could not resolve target "
                f"({exc})"
            )
            return

        command = PoseCommand(
            target=target,
            relative=relative,
            source_topic=source_topic,
        )
        with self._condition:
            self._pending_command = command
            self._condition.notify()

    def _target_from_relative_pose(self, message: PoseStamped) -> PoseTarget:
        frame_id = message.header.frame_id.strip().lstrip("/")
        if not frame_id or frame_id == self._base_frame:
            return self._target_from_pose(message)
        raise ValueError(
            f"relative offsets must be published in '{self._base_frame}', not "
            f"'{frame_id}'"
        )

    @staticmethod
    def _target_from_pose(message: PoseStamped) -> PoseTarget:
        position = message.pose.position
        orientation = message.pose.orientation
        return PoseTarget(
            x=position.x,
            y=position.y,
            z=position.z,
            yaw=yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )

    def _run_worker(self) -> None:
        while rclpy.ok():
            with self._condition:
                while self._pending_command is None and not self._stop_requested:
                    self._condition.wait()
                if self._stop_requested:
                    return
                command = self._pending_command
                self._pending_command = None
                self._active_command = command

            if command is None:
                continue

            self.get_logger().info(
                f"Executing {'relative' if command.relative else 'absolute'} "
                f"PoseStamped command from {command.source_topic}"
            )
            if command.relative:
                success, message, _ = self._controller.move_relative(
                    dx=command.target.x,
                    dy=command.target.y,
                    dz=command.target.z,
                    dyaw=command.target.yaw,
                    position_tolerance=self._position_tolerance,
                    yaw_tolerance=self._yaw_tolerance,
                    timeout_sec=self._timeout,
                    cancel_requested=self._has_new_command_or_stop,
                )
            else:
                success, message, _ = self._controller.move_absolute(
                    target=command.target,
                    position_tolerance=self._position_tolerance,
                    yaw_tolerance=self._yaw_tolerance,
                    timeout_sec=self._timeout,
                    cancel_requested=self._has_new_command_or_stop,
                )

            with self._condition:
                self._active_command = None
                interrupted = self._pending_command is not None or self._stop_requested

            if interrupted:
                self.get_logger().info(
                    "PoseStamped command superseded by a newer target"
                )
            elif success:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)

    def _has_new_command_or_stop(self) -> bool:
        with self._condition:
            return self._pending_command is not None or self._stop_requested


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PoseStampedEndEffectorControl()
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
