#!/usr/bin/env python3

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
import yaml


def quaternion_from_rpy(roll, pitch, yaw):
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


def pose_values(raw_pose):
    pose = list(raw_pose or [])
    pose.extend([0.0] * (6 - len(pose)))
    return [float(value) for value in pose[:6]]


class PickPlaceScenePublisher(Node):
    def __init__(self):
        super().__init__("pick_place_scene_publisher")
        self.declare_parameter("scene_config", "")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_period", 1.0)

        config_path = Path(str(self.get_parameter("scene_config").value))
        if not config_path.is_file():
            raise FileNotFoundError(f"Scene config does not exist: {config_path}")

        self.scene = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        self.frame_id = str(self.scene.get("frame_id") or self.get_parameter("frame_id").value)
        self.broadcaster = StaticTransformBroadcaster(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(String, "pick_place_scene/objects", qos)

        self.transforms = self._transforms_from_scene()
        self.broadcaster.sendTransform(self.transforms)
        period = max(float(self.get_parameter("publish_period").value), 0.1)
        self.create_timer(period, self._publish_scene)
        self._publish_scene()
        self.get_logger().info(
            f"Published {len(self.transforms)} pick-place scene frame(s) from {config_path}"
        )

    def _transforms_from_scene(self):
        transforms = []
        entries = []
        table = self.scene.get("table")
        if isinstance(table, dict):
            entries.append(("table", table))
        for key, kind in (
            ("cubes", "cube"),
            ("place_targets", "place_target"),
            ("charuco_boards", "charuco_board"),
        ):
            for item in self.scene.get(key, []) or []:
                entries.append((kind, item))

        for kind, item in entries:
            frame_id = str(item.get("frame_id") or item.get("name") or kind)
            transform = TransformStamped()
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = frame_id
            x, y, z, roll, pitch, yaw = pose_values(item.get("pose"))
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            transforms.append(transform)
        return transforms

    def _publish_scene(self):
        now = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = now
        self.broadcaster.sendTransform(self.transforms)

        message = String()
        message.data = json.dumps(self._scene_payload(), separators=(",", ":"))
        self.scene_pub.publish(message)

    def _scene_payload(self):
        return {
            "frame_id": self.frame_id,
            "cubes": self.scene.get("cubes", []) or [],
            "place_targets": self.scene.get("place_targets", []) or [],
            "charuco_boards": self.scene.get("charuco_boards", []) or [],
        }


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceScenePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
