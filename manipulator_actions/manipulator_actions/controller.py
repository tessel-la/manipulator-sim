"""ROS-facing MoveIt Servo controller primitives."""

from __future__ import annotations

import time
from typing import Callable, Optional, Tuple

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from manipulator_actions.motion_math import (
    MotionLimits,
    PoseTarget,
    relative_target,
    servo_command,
    target_error,
    wrap_angle,
    yaw_from_quaternion,
)


FeedbackCallback = Callable[[float, float, str], None]


class ServoMotionController:
    """Closed-loop Cartesian motion helper backed by MoveIt Servo twist commands."""

    def __init__(
        self,
        node: Node,
        base_frame: str = "panda_link0",
        ee_frame: str = "panda_link8",
        twist_topic: str = "/servo_node/delta_twist_cmds",
        start_servo_service: str = "/servo_node/start_servo",
        publish_hz: float = 20.0,
        limits: Optional[MotionLimits] = None,
    ) -> None:
        self._node = node
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.publish_hz = publish_hz
        self.limits = limits or MotionLimits()
        self._publisher = node.create_publisher(TwistStamped, twist_topic, 10)
        self._start_client = node.create_client(Trigger, start_servo_service)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    def start_servo(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        if not self._start_client.wait_for_service(timeout_sec=timeout_sec):
            return False, "Servo start service is not available"

        future = self._start_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not future.done():
            return False, "Timed out waiting for Servo start response"

        response = future.result()
        if response is None:
            return False, "Servo start service returned no response"
        if not response.success:
            return False, response.message or "Servo refused to start"
        return True, response.message or "Servo started"

    def current_pose(self) -> PoseTarget:
        transform = self._tf_buffer.lookup_transform(
            self.base_frame,
            self.ee_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return PoseTarget(
            x=translation.x,
            y=translation.y,
            z=translation.z,
            yaw=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def move_absolute(
        self,
        target: PoseTarget,
        position_tolerance: float,
        yaw_tolerance: float,
        timeout_sec: float,
        feedback: Optional[FeedbackCallback] = None,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> Tuple[bool, str, PoseTarget]:
        ok, message = self.start_servo()
        if not ok:
            return False, message, PoseTarget(0.0, 0.0, 0.0, 0.0)
        return self._servo_to_target(
            target=target,
            position_tolerance=position_tolerance,
            yaw_tolerance=yaw_tolerance,
            timeout_sec=timeout_sec,
            feedback=feedback,
            cancel_requested=cancel_requested,
        )

    def move_relative(
        self,
        dx: float,
        dy: float,
        dz: float,
        dyaw: float,
        position_tolerance: float,
        yaw_tolerance: float,
        timeout_sec: float,
        feedback: Optional[FeedbackCallback] = None,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> Tuple[bool, str, PoseTarget]:
        try:
            start = self.current_pose()
        except TransformException as exc:
            return False, f"Could not read current pose: {exc}", PoseTarget(0.0, 0.0, 0.0, 0.0)
        target = relative_target(start, dx, dy, dz, dyaw)
        return self.move_absolute(
            target=target,
            position_tolerance=position_tolerance,
            yaw_tolerance=yaw_tolerance,
            timeout_sec=timeout_sec,
            feedback=feedback,
            cancel_requested=cancel_requested,
        )

    def halt(self, repeats: int = 5) -> None:
        for _ in range(repeats):
            self._publisher.publish(self._twist(0.0, 0.0, 0.0, 0.0))
            time.sleep(0.02)

    def _servo_to_target(
        self,
        target: PoseTarget,
        position_tolerance: float,
        yaw_tolerance: float,
        timeout_sec: float,
        feedback: Optional[FeedbackCallback],
        cancel_requested: Optional[Callable[[], bool]],
    ) -> Tuple[bool, str, PoseTarget]:
        deadline = time.monotonic() + timeout_sec
        last_pose = PoseTarget(0.0, 0.0, 0.0, 0.0)
        period = 1.0 / self.publish_hz

        while rclpy.ok() and time.monotonic() < deadline:
            if cancel_requested is not None and cancel_requested():
                self.halt()
                return False, "Motion canceled", last_pose

            try:
                current = self.current_pose()
            except TransformException as exc:
                self.halt()
                return False, f"Could not read current pose: {exc}", last_pose

            last_pose = current
            vx, vy, vz, wz, distance, yaw_error = servo_command(
                current=current,
                target=target,
                limits=self.limits,
                position_tolerance=position_tolerance,
                yaw_tolerance=yaw_tolerance,
            )

            if feedback is not None:
                feedback(distance, yaw_error, "moving")

            if distance <= position_tolerance and yaw_error <= yaw_tolerance:
                self.halt()
                return True, "Target reached", current

            self._publisher.publish(self._twist(vx, vy, vz, wz))
            time.sleep(period)

        self.halt()
        try:
            last_pose = self.current_pose()
        except TransformException:
            pass
        _, _, _, remaining_distance, remaining_yaw = target_error(last_pose, target)
        return (
            False,
            (
                "Timed out before reaching target "
                f"(distance={remaining_distance:.3f} m, yaw={abs(wrap_angle(remaining_yaw)):.3f} rad)"
            ),
            last_pose,
        )

    def _twist(self, vx: float, vy: float, vz: float, wz: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = wz
        return msg
