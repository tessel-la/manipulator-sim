#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


class GazeboWristCameraFollower(Node):
    def __init__(self):
        super().__init__("gazebo_wrist_camera_follower")

        self.declare_parameter("source_frame", "panda_hand")
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("entity_name", "wrist_camera")
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("offset_x", 0.055)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.055)
        self.declare_parameter("position_epsilon", 0.002)
        self.declare_parameter("orientation_epsilon", 0.003)

        self.source_frame = self.get_parameter("source_frame").value
        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.entity_name = self.get_parameter("entity_name").value
        update_rate = float(self.get_parameter("update_rate").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_pose_client = self.create_client(
            SetEntityPose, "/world/default/set_pose"
        )
        self.pending_request = None
        self.pending_since = None
        self.last_position = None
        self.last_orientation = None
        self.warned_about_service = False

        self.timer = self.create_timer(1.0 / update_rate, self._update_pose)
        self.get_logger().info(
            f"Driving Gazebo entity '{self.entity_name}' from ROS TF '{self.source_frame}'"
        )

    def _update_pose(self):
        if not self.set_pose_client.service_is_ready():
            if not self.warned_about_service:
                self.get_logger().info("Waiting for /world/default/set_pose")
                self.warned_about_service = True
            return

        if self.pending_request is not None and not self.pending_request.done():
            if self.get_clock().now().nanoseconds - self.pending_since < 500_000_000:
                return
        self.pending_request = None
        self.pending_since = None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.source_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        offset = self._rotate_vector(
            transform.transform.rotation,
            [
                self.get_parameter("offset_x").value,
                self.get_parameter("offset_y").value,
                self.get_parameter("offset_z").value,
            ],
        )
        position = [
            transform.transform.translation.x + offset[0],
            transform.transform.translation.y + offset[1],
            transform.transform.translation.z + offset[2],
        ]
        orientation = transform.transform.rotation

        if not self._pose_changed(position, orientation):
            return

        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = position[0]
        request.pose.position.y = position[1]
        request.pose.position.z = position[2]
        request.pose.orientation = orientation

        self.pending_request = self.set_pose_client.call_async(request)
        self.pending_since = self.get_clock().now().nanoseconds
        self.last_position = position
        self.last_orientation = orientation

    def _pose_changed(self, position, orientation):
        if self.last_position is None or self.last_orientation is None:
            return True

        position_epsilon = self.get_parameter("position_epsilon").value
        orientation_epsilon = self.get_parameter("orientation_epsilon").value
        dx = position[0] - self.last_position[0]
        dy = position[1] - self.last_position[1]
        dz = position[2] - self.last_position[2]
        if dx * dx + dy * dy + dz * dz > position_epsilon * position_epsilon:
            return True

        dot = abs(
            orientation.x * self.last_orientation.x
            + orientation.y * self.last_orientation.y
            + orientation.z * self.last_orientation.z
            + orientation.w * self.last_orientation.w
        )
        return 1.0 - min(dot, 1.0) > orientation_epsilon

    def _rotate_vector(self, quaternion, vector):
        x, y, z = vector
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w

        uv = [qy * z - qz * y, qz * x - qx * z, qx * y - qy * x]
        uuv = [
            qy * uv[2] - qz * uv[1],
            qz * uv[0] - qx * uv[2],
            qx * uv[1] - qy * uv[0],
        ]
        return [
            x + 2.0 * (qw * uv[0] + uuv[0]),
            y + 2.0 * (qw * uv[1] + uuv[1]),
            z + 2.0 * (qw * uv[2] + uuv[2]),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = GazeboWristCameraFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
