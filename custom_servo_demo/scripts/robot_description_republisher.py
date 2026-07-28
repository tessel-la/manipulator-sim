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
        self.declare_parameter(
            "additional_topic_names", rclpy.Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter(
            "additional_robot_descriptions", rclpy.Parameter.Type.STRING_ARRAY
        )

        self._description = (
            self.get_parameter("robot_description").get_parameter_value().string_value
        )
        self._topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        additional_topic_names = list(
            self.get_parameter("additional_topic_names").value
        )
        additional_descriptions = list(
            self.get_parameter("additional_robot_descriptions").value
        )
        if len(additional_topic_names) != len(additional_descriptions):
            raise ValueError(
                "additional_topic_names and additional_robot_descriptions "
                "must have the same length"
            )
        publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publishers = []
        self._messages = []
        self._qos_profile = qos_profile
        self._timer = None
        self._subscription = None

        for topic_name, description in zip(
            additional_topic_names, additional_descriptions
        ):
            self._add_publication(topic_name, description)

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
            self._publish_all()
            if publish_period > 0.0 and self._messages:
                self._timer = self.create_timer(publish_period, self._publish_all)
            return

        self._add_publication(self._topic_name, self._description)
        self._start_publishing(publish_period)

    def _store_description(self, message):
        if not message.data:
            return

        publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )
        if self._subscription is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        self._add_publication(self._topic_name, message.data)
        self._start_publishing(publish_period)

    def _add_publication(self, topic_name, description):
        publisher = self.create_publisher(String, topic_name, self._qos_profile)
        message = String()
        message.data = description
        self._publishers.append(publisher)
        self._messages.append(message)

    def _start_publishing(self, publish_period):
        self._publish_all()
        if publish_period <= 0.0:
            self.get_logger().info(
                f"Published {len(self._messages)} robot description topic(s) "
                "with transient-local QoS"
            )
            return

        if self._timer is None:
            self._timer = self.create_timer(publish_period, self._publish_all)
        self.get_logger().info(
            f"Publishing {len(self._messages)} robot description topic(s) "
            f"every {publish_period:.2f}s"
        )

    def _publish_all(self):
        for publisher, message in zip(self._publishers, self._messages):
            publisher.publish(message)


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
