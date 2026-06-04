import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _camera_actions(context):
    arm_count = int(LaunchConfiguration("arm_count").perform(context))
    if arm_count < 1:
        raise RuntimeError("arm_count must be at least 1")

    enabled = LaunchConfiguration("enabled").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    if not enabled:
        return []

    if arm_count > 3:
        raise RuntimeError("gazebo_wrist_camera.launch.py supports up to 3 cameras")

    arm_prefix = LaunchConfiguration("arm_prefix").perform(context).strip("/")
    if not arm_prefix and arm_count > 1:
        raise RuntimeError("arm_prefix must be set when arm_count is greater than 1")

    use_prefixed_frames = (
        LaunchConfiguration("use_prefixed_frames").perform(context).lower()
        in ("1", "true", "yes", "on")
    )

    world_path = os.path.join(
        get_package_share_directory("custom_servo_demo"),
        "worlds",
        "wrist_camera_world.sdf",
    )

    gz_sim = launch.actions.ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_path],
        output="screen",
    )

    set_pose_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_set_pose_bridge",
        arguments=[
            "/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output="screen",
    )

    actions = [gz_sim, set_pose_bridge]
    camera_configs = []
    if arm_prefix:
        camera_configs = [
            {
                "namespace": f"{arm_prefix}_{index}",
                "image_topic": f"/{arm_prefix}_{index}/wrist_camera/image_raw",
                "camera_info_output": f"/{arm_prefix}_{index}/wrist_camera/camera_info",
                "entity_name": f"wrist_camera_{arm_prefix}_{index}",
                "source_frame": (
                    f"{arm_prefix}_{index}_panda_hand"
                    if use_prefixed_frames
                    else "panda_hand"
                ),
            }
            for index in range(1, arm_count + 1)
        ]
    else:
        camera_configs = [
            {
                "namespace": None,
                "image_topic": "/wrist_camera/image_raw",
                "camera_info_output": "/wrist_camera/camera_info",
                "entity_name": "wrist_camera",
                "source_frame": "panda_hand",
            }
        ]

    for camera in camera_configs:
        namespace = camera["namespace"]
        image_topic = camera["image_topic"]
        camera_info_topic = f"{image_topic}/camera_info"

        actions.extend(
            [
                launch_ros.actions.Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    namespace=namespace,
                    name="wrist_camera_image_bridge",
                    arguments=[
                        f"{image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                    ],
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    namespace=namespace,
                    name="wrist_camera_info_bridge",
                    arguments=[
                        f"{camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "--ros-args",
                        "-r",
                        f"{camera_info_topic}:={camera['camera_info_output']}",
                    ],
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="custom_servo_demo",
                    executable="gazebo_wrist_camera_follower",
                    namespace=namespace,
                    name="gazebo_wrist_camera_follower",
                    parameters=[
                        {
                            "source_frame": camera["source_frame"],
                            "fixed_frame": "world",
                            "entity_name": camera["entity_name"],
                            "update_rate": 2.0,
                            "offset_x": 0.055,
                            "offset_y": 0.0,
                            "offset_z": 0.055,
                            "position_epsilon": 0.002,
                            "orientation_epsilon": 0.003,
                        }
                    ],
                    output="screen",
                ),
            ]
        )

    return actions


def generate_launch_description():

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_count",
                default_value="1",
                description="Number of wrist cameras to launch",
            ),
            DeclareLaunchArgument(
                "arm_prefix",
                default_value="",
                description="Optional prefix used to create namespaces such as arm_1 and arm_2",
            ),
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="Launch Gazebo and namespaced wrist camera bridges",
            ),
            DeclareLaunchArgument(
                "use_prefixed_frames",
                default_value="false",
                description="Use frames such as arm_1_panda_hand for camera followers",
            ),
            OpaqueFunction(function=_camera_actions),
        ]
    )
