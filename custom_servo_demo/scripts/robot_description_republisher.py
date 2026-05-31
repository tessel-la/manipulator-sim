#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionRepublisher(Node):
    def __init__(self):
        super().__init__("robot_description_republisher")

        self.declare_parameter("robot_description", "")
        self.declare_parameter("topic_name", "/robot_description")
        self.declare_parameter("publish_period", 1.0)

        self._description = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        self._topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(String, self._topic_name, qos_profile)
        self._qos_profile = qos_profile
        self._timer = None
        self._message = None
        self._subscription = None

        if not self._description:
            self.get_logger().info(
                f"robot_description parameter is empty; waiting for {self._topic_name}"
            )
            self._subscription = self.create_subscription(
                String,
                self._topic_name,
                self._store_description,
                self._qos_profile,
            )
            return

        self._start_publishing(self._description, publish_period)

    def _store_description(self, message):
        if not message.data:
            return

        publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        self._start_publishing(message.data, publish_period)

    def _start_publishing(self, description, publish_period):
        self._message = String()
        self._message.data = description
        self._timer = self.create_timer(publish_period, self._publish_description)
        self._publish_description()
        self.get_logger().info(
            f"Publishing robot_description on {self._topic_name} every {publish_period:.2f}s"
        )

    def _publish_description(self):
        if self._message is None:
            return
        self._publisher.publish(self._message)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
