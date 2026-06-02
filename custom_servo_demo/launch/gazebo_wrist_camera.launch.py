import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory("custom_servo_demo"),
        "worlds",
        "wrist_camera_world.sdf",
    )

    gz_sim = launch.actions.ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_path],
        output="screen",
    )

    image_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_camera_image_bridge",
        arguments=[
            "/wrist_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    camera_info_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrist_camera_info_bridge",
        arguments=[
            "/wrist_camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "--ros-args",
            "-r",
            "/wrist_camera/image_raw/camera_info:=/wrist_camera/camera_info",
        ],
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

    camera_follower = launch_ros.actions.Node(
        package="custom_servo_demo",
        executable="gazebo_wrist_camera_follower",
        name="gazebo_wrist_camera_follower",
        parameters=[
            {
                "source_frame": "panda_hand",
                "fixed_frame": "world",
                "entity_name": "wrist_camera",
                "update_rate": 2.0,
                "offset_x": 0.055,
                "offset_y": 0.0,
                "offset_z": 0.055,
                "position_epsilon": 0.002,
                "orientation_epsilon": 0.003,
            }
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            gz_sim,
            image_bridge,
            camera_info_bridge,
            set_pose_bridge,
            camera_follower,
        ]
    )
