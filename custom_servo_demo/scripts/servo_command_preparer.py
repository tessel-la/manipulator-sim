#!/usr/bin/env python3

from __future__ import annotations

from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ServoCommandPreparer(Node):
    """Select Twist command mode and unpause a MoveIt Servo node."""

    def __init__(self):
        super().__init__("servo_command_preparer")

        self.declare_parameter(
            "switch_command_type_service", "servo_node/switch_command_type"
        )
        self.declare_parameter("pause_servo_service", "servo_node/pause_servo")
        self.declare_parameter("command_type", 1)
        self.declare_parameter("timeout", 10.0)

        self._command_type_client = self.create_client(
            ServoCommandType,
            self.get_parameter("switch_command_type_service").value,
        )
        self._pause_client = self.create_client(
            SetBool,
            self.get_parameter("pause_servo_service").value,
        )

    def prepare(self) -> bool:
        timeout = float(self.get_parameter("timeout").value)
        command_type = int(self.get_parameter("command_type").value)

        if not self._command_type_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Servo command type service is not available")
            return False

        command_request = ServoCommandType.Request()
        command_request.command_type = command_type
        command_future = self._command_type_client.call_async(command_request)
        rclpy.spin_until_future_complete(self, command_future, timeout_sec=timeout)
        command_response = command_future.result()
        if command_response is None or not command_response.success:
            self.get_logger().error("Servo refused Twist command mode")
            return False

        if not self._pause_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("Servo pause service is not available")
            return False

        pause_request = SetBool.Request()
        pause_request.data = False
        pause_future = self._pause_client.call_async(pause_request)
        rclpy.spin_until_future_complete(self, pause_future, timeout_sec=timeout)
        pause_response = pause_future.result()
        if pause_response is None or not pause_response.success:
            message = "" if pause_response is None else pause_response.message
            self.get_logger().error(message or "Servo refused to unpause")
            return False

        self.get_logger().info(pause_response.message or "Servo ready")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandPreparer()
    try:
        success = node.prepare()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
