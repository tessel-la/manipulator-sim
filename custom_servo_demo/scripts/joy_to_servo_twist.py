#!/usr/bin/env python3

from __future__ import annotations

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyToServoTwist(Node):
    """Translate sensor_msgs/Joy input into MoveIt Servo TwistStamped commands."""

    def __init__(self):
        super().__init__("joy_to_servo_twist")

        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("twist_topic", "servo_node/delta_twist_cmds")
        self.declare_parameter("frame_id", "panda_link0")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("enable_button", -1)
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_linear_z", 4)
        self.declare_parameter("axis_angular_z", 3)
        self.declare_parameter("linear_x_scale", 0.2)
        self.declare_parameter("linear_y_scale", 0.2)
        self.declare_parameter("linear_z_scale", 0.2)
        self.declare_parameter("angular_z_scale", 0.8)

        self._joy_topic = self.get_parameter("joy_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._stale_timeout = max(float(self.get_parameter("stale_timeout").value), 0.0)
        self._deadzone = min(max(float(self.get_parameter("deadzone").value), 0.0), 0.99)
        self._enable_button = int(self.get_parameter("enable_button").value)
        self._axis_linear_x = int(self.get_parameter("axis_linear_x").value)
        self._axis_linear_y = int(self.get_parameter("axis_linear_y").value)
        self._axis_linear_z = int(self.get_parameter("axis_linear_z").value)
        self._axis_angular_z = int(self.get_parameter("axis_angular_z").value)
        self._linear_x_scale = float(self.get_parameter("linear_x_scale").value)
        self._linear_y_scale = float(self.get_parameter("linear_y_scale").value)
        self._linear_z_scale = float(self.get_parameter("linear_z_scale").value)
        self._angular_z_scale = float(self.get_parameter("angular_z_scale").value)

        twist_topic = self.get_parameter("twist_topic").value
        publish_hz = float(self.get_parameter("publish_hz").value)
        publish_period = 1.0 / max(publish_hz, 1.0)

        self._publisher = self.create_publisher(TwistStamped, twist_topic, 10)
        self._subscription = self.create_subscription(
            Joy,
            self._joy_topic,
            self._store_joy_command,
            10,
        )
        self._timer = self.create_timer(publish_period, self._publish_command)
        self._last_joy_time = None
        self._last_twist = self._zero_twist()
        self._last_enabled = False
        self._was_publishing = False

        enable_text = (
            "always enabled"
            if self._enable_button < 0
            else f"enable button {self._enable_button}"
        )
        self.get_logger().info(
            "Relaying Joy commands from "
            f"{self._joy_topic} to {twist_topic} ({enable_text})"
        )

    def _store_joy_command(self, message: Joy):
        self._last_joy_time = self.get_clock().now()
        self._last_enabled = self._is_enabled(message)
        if self._last_enabled:
            self._last_twist = self._twist_from_joy(message)
        else:
            self._last_twist = self._zero_twist()

    def _publish_command(self):
        if self._last_joy_time is None:
            return

        stale = (
            (self.get_clock().now() - self._last_joy_time).nanoseconds
            / 1_000_000_000.0
            > self._stale_timeout
        )
        command = (
            self._zero_twist()
            if stale or not self._last_enabled
            else self._last_twist
        )
        has_motion = self._has_motion(command)

        if not has_motion and not self._was_publishing:
            return

        command.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(command)
        self._was_publishing = has_motion

    def _is_enabled(self, message: Joy) -> bool:
        if self._enable_button < 0:
            return True
        if self._enable_button >= len(message.buttons):
            return False
        return message.buttons[self._enable_button] != 0

    def _twist_from_joy(self, message: Joy) -> TwistStamped:
        twist = self._zero_twist()
        twist.twist.linear.x = (
            self._axis_value(message, self._axis_linear_x) * self._linear_x_scale
        )
        twist.twist.linear.y = (
            self._axis_value(message, self._axis_linear_y) * self._linear_y_scale
        )
        twist.twist.linear.z = (
            self._axis_value(message, self._axis_linear_z) * self._linear_z_scale
        )
        twist.twist.angular.z = (
            self._axis_value(message, self._axis_angular_z) * self._angular_z_scale
        )
        return twist

    def _axis_value(self, message: Joy, axis_index: int) -> float:
        if axis_index < 0 or axis_index >= len(message.axes):
            return 0.0

        value = max(min(message.axes[axis_index], 1.0), -1.0)
        if abs(value) <= self._deadzone:
            return 0.0

        magnitude = (abs(value) - self._deadzone) / max(1.0 - self._deadzone, 0.001)
        return magnitude if value > 0.0 else -magnitude

    def _zero_twist(self) -> TwistStamped:
        twist = TwistStamped()
        twist.header.frame_id = self._frame_id
        return twist

    @staticmethod
    def _has_motion(message: TwistStamped) -> bool:
        twist = message.twist
        return any(
            abs(value) > 1e-6
            for value in (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServoTwist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
