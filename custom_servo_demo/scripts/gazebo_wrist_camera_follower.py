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
        self.declare_parameter("offset_roll", 0.0)
        self.declare_parameter("offset_pitch", 0.0)
        self.declare_parameter("offset_yaw", 0.0)
        self.declare_parameter("look_at_frame", "")
        self.declare_parameter("look_at_x", 0.45)
        self.declare_parameter("look_at_y", 0.0)
        self.declare_parameter("look_at_z", 0.04)
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
        orientation = self._orientation_for_camera(position, transform.transform.rotation)

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

    def _orientation_with_offset(self, orientation):
        offset = self._quaternion_from_rpy(
            self.get_parameter("offset_roll").value,
            self.get_parameter("offset_pitch").value,
            self.get_parameter("offset_yaw").value,
        )
        x, y, z, w = self._quaternion_multiply(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            offset,
        )
        orientation.x = x
        orientation.y = y
        orientation.z = z
        orientation.w = w
        return orientation

    def _orientation_for_camera(self, position, source_orientation):
        target = self._look_at_target()
        if target is None:
            return self._orientation_with_offset(source_orientation)

        forward = [
            target[0] - position[0],
            target[1] - position[1],
            target[2] - position[2],
        ]
        orientation = self._quaternion_from_forward(forward)
        if orientation is None:
            return self._orientation_with_offset(source_orientation)
        return orientation

    def _look_at_target(self):
        frame = str(self.get_parameter("look_at_frame").value).strip().lstrip("/")
        if frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    frame,
                    Time(),
                )
                translation = transform.transform.translation
                return [translation.x, translation.y, translation.z]
            except (LookupException, ConnectivityException, ExtrapolationException):
                return None

        return [
            self.get_parameter("look_at_x").value,
            self.get_parameter("look_at_y").value,
            self.get_parameter("look_at_z").value,
        ]

    def _quaternion_from_forward(self, forward):
        x_axis = self._normalize(forward)
        if x_axis is None:
            return None

        up_hint = [0.0, 0.0, 1.0]
        z_axis = self._reject(up_hint, x_axis)
        if z_axis is None:
            z_axis = self._reject([0.0, 1.0, 0.0], x_axis)
        if z_axis is None:
            return None

        y_axis = self._cross(z_axis, x_axis)
        return self._quaternion_from_matrix(
            [
                [x_axis[0], y_axis[0], z_axis[0]],
                [x_axis[1], y_axis[1], z_axis[1]],
                [x_axis[2], y_axis[2], z_axis[2]],
            ]
        )

    def _normalize(self, vector):
        import math

        norm = math.sqrt(sum(value * value for value in vector))
        if norm < 1e-9:
            return None
        return [value / norm for value in vector]

    def _reject(self, vector, axis):
        dot = sum(value * axis_value for value, axis_value in zip(vector, axis))
        return self._normalize(
            [value - dot * axis_value for value, axis_value in zip(vector, axis)]
        )

    def _cross(self, first, second):
        return [
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        ]

    def _quaternion_from_matrix(self, matrix):
        import math
        from geometry_msgs.msg import Quaternion

        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (m21 - m12) / scale
            y = (m02 - m20) / scale
            z = (m10 - m01) / scale
        elif m00 > m11 and m00 > m22:
            scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / scale
            x = 0.25 * scale
            y = (m01 + m10) / scale
            z = (m02 + m20) / scale
        elif m11 > m22:
            scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / scale
            x = (m01 + m10) / scale
            y = 0.25 * scale
            z = (m12 + m21) / scale
        else:
            scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (m10 - m01) / scale
            x = (m02 + m20) / scale
            y = (m12 + m21) / scale
            z = 0.25 * scale

        quaternion = Quaternion()
        quaternion.x = x
        quaternion.y = y
        quaternion.z = z
        quaternion.w = w
        return quaternion

    def _quaternion_from_rpy(self, roll, pitch, yaw):
        import math

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _quaternion_multiply(self, first, second):
        ax, ay, az, aw = first
        bx, by, bz, bw = second
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )


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
