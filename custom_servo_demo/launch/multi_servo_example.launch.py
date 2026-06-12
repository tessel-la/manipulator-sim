import copy
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


ARM_JOINTS = [f"panda_joint{index}" for index in range(1, 8)]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]


def _prefix_name(prefix, value):
    if not value or value == "world":
        return value
    if value.startswith(prefix):
        return value
    return f"{prefix}{value}"


def _prefix_urdf_tree(element, prefix):
    for node in element.iter():
        if node.tag in ("link", "joint") and "name" in node.attrib:
            node.set("name", _prefix_name(prefix, node.get("name")))
        elif node.tag in ("parent", "child") and "link" in node.attrib:
            node.set("link", _prefix_name(prefix, node.get("link")))
        elif node.tag == "mimic" and "joint" in node.attrib:
            node.set("joint", _prefix_name(prefix, node.get("joint")))
        elif node.tag == "ros2_control" and "name" in node.attrib:
            node.set("name", _prefix_name(prefix, node.get("name")))


def _xml_string(element):
    return ET.tostring(element, encoding="unicode")


def _prefixed_urdf(robot_description, namespace, xyz):
    prefix = f"{namespace}_"
    robot = ET.fromstring(robot_description)
    _prefix_urdf_tree(robot, prefix)
    robot.set("name", namespace)

    world_link = ET.Element("link", {"name": "world"})
    base_joint = ET.Element(
        "joint",
        {
            "name": f"{namespace}_world_joint",
            "type": "fixed",
        },
    )
    ET.SubElement(base_joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    ET.SubElement(base_joint, "parent", {"link": "world"})
    ET.SubElement(base_joint, "child", {"link": f"{namespace}_panda_link0"})

    robot.insert(0, base_joint)
    robot.insert(0, world_link)
    return _xml_string(robot)


def _aggregate_urdf(robot_description, namespaces, poses):
    aggregate = ET.Element("robot", {"name": "multi_panda"})
    ET.SubElement(aggregate, "link", {"name": "world"})

    for namespace, xyz in zip(namespaces, poses):
        arm = ET.fromstring(_prefixed_urdf(robot_description, namespace, xyz))
        for child in list(arm):
            if child.tag == "link" and child.get("name") == "world":
                continue
            aggregate.append(copy.deepcopy(child))

    return _xml_string(aggregate)


def _prefixed_srdf(robot_description_semantic, namespace):
    prefix = f"{namespace}_"
    semantic = ET.fromstring(robot_description_semantic)
    semantic.set("name", namespace)
    for parent in semantic.iter():
        for child in list(parent):
            if child.tag == "virtual_joint":
                parent.remove(child)

    for node in semantic.iter():
        if node.tag in ("link", "joint", "passive_joint") and "name" in node.attrib:
            node.set("name", _prefix_name(prefix, node.get("name")))
        elif node.tag == "chain":
            node.set("base_link", _prefix_name(prefix, node.get("base_link")))
            node.set("tip_link", _prefix_name(prefix, node.get("tip_link")))
        elif node.tag == "virtual_joint":
            node.set("child_link", _prefix_name(prefix, node.get("child_link")))
        elif node.tag == "disable_collisions":
            node.set("link1", _prefix_name(prefix, node.get("link1")))
            node.set("link2", _prefix_name(prefix, node.get("link2")))
        elif node.tag == "end_effector":
            node.set("parent_link", _prefix_name(prefix, node.get("parent_link")))
    return _xml_string(semantic)


def _prefixed_kinematics(robot_description_kinematics):
    return copy.deepcopy(robot_description_kinematics)


def _prefixed_joint_limits(joint_limits, namespace):
    prefix = f"{namespace}_"
    limits = copy.deepcopy(joint_limits)
    robot_description_planning = limits.get("robot_description_planning", {})
    joint_limits_map = robot_description_planning.get("joint_limits", {})
    robot_description_planning["joint_limits"] = {
        _prefix_name(prefix, name): value for name, value in joint_limits_map.items()
    }
    limits["robot_description_planning"] = robot_description_planning
    return limits


def _servo_params(base_servo_params, namespace):
    servo_params = copy.deepcopy(base_servo_params)
    moveit_servo = servo_params["moveit_servo"]
    moveit_servo["joint_topic"] = "/joint_states"
    moveit_servo["command_out_topic"] = (
        f"/{namespace}/panda_arm_controller/joint_trajectory"
    )
    moveit_servo["move_group_name"] = "panda_arm"
    return servo_params


def _controller_config_path(namespace):
    prefix = f"{namespace}_"
    data = {
        f"/{namespace}/controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "panda_arm_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                },
                "panda_hand_controller": {
                    "type": "position_controllers/GripperActionController"
                },
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
            }
        },
        f"/{namespace}/panda_arm_controller": {
            "ros__parameters": {
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "joints": [_prefix_name(prefix, joint) for joint in ARM_JOINTS],
                "allow_nonzero_velocity_at_trajectory_end": True,
            }
        },
        f"/{namespace}/panda_hand_controller": {
            "ros__parameters": {
                "joint": _prefix_name(prefix, "panda_finger_joint1"),
            }
        },
    }

    config_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"{namespace}_controllers_",
        suffix=".yaml",
        delete=False,
    )
    with config_file:
        yaml.safe_dump(data, config_file)
    return config_file.name


def _arm_poses(arm_count, spacing):
    center = (arm_count - 1) / 2.0
    return [f"0 {((index - center) * spacing):.3f} 0" for index in range(arm_count)]


def _arm_actions(context):
    arm_count = int(LaunchConfiguration("arm_count").perform(context))
    if arm_count < 1:
        raise RuntimeError("arm_count must be at least 1")

    arm_prefix = LaunchConfiguration("arm_prefix").perform(context).strip("/")
    if not arm_prefix:
        raise RuntimeError("arm_prefix must not be empty")

    arm_spacing = float(LaunchConfiguration("arm_spacing").perform(context))
    use_joy_teleop = LaunchConfiguration("use_joy_teleop")
    use_pose_stamped_control = LaunchConfiguration("use_pose_stamped_control")
    launch_action_servers = LaunchConfiguration("launch_action_servers")
    prepare_servo = LaunchConfiguration("prepare_servo")
    behavior_tree_name = LaunchConfiguration("behavior_tree_name").perform(context).strip()
    behavior_tree_timeout = LaunchConfiguration("behavior_tree_timeout")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_camera = LaunchConfiguration("use_gazebo_camera")
    launch_behavior_tree = behavior_tree_name.lower() not in (
        "",
        "0",
        "false",
        "no",
        "none",
        "off",
    )

    namespaces = [f"{arm_prefix}_{index}" for index in range(1, arm_count + 1)]
    poses = _arm_poses(arm_count, arm_spacing)

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .joint_limits(file_path="config/hard_joint_limits.yaml")
        .robot_description_kinematics()
        .to_moveit_configs()
    )
    robot_description = moveit_config.robot_description["robot_description"]
    robot_description_semantic = moveit_config.robot_description_semantic[
        "robot_description_semantic"
    ]
    aggregate_description = _aggregate_urdf(robot_description, namespaces, poses)
    aggregate_robot_description = {"robot_description": aggregate_description}

    base_servo_params = {
        "moveit_servo": ParameterBuilder("custom_servo_demo")
        .yaml("config/panda_simulated_config.yaml")
        .to_dict()
    }
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "panda_arm"}

    rviz_config_file = os.path.join(
        get_package_share_directory("custom_servo_demo"),
        "config",
        "multi_demo_rviz_config.rviz",
    )
    actions = [
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[aggregate_robot_description],
            condition=IfCondition(use_rviz),
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="multi_robot_state_publisher",
            parameters=[aggregate_robot_description],
            output="screen",
        ),
        launch_ros.actions.Node(
            package="custom_servo_demo",
            executable="robot_description_republisher",
            name="multi_robot_description_republisher",
            parameters=[
                aggregate_robot_description,
                {"topic_name": "/robot_description", "publish_period": 0.0},
            ],
            output="screen",
        ),
    ]

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("custom_servo_demo"),
                "launch",
                "gazebo_wrist_camera.launch.py",
            )
        ),
        launch_arguments={
            "arm_count": str(arm_count),
            "arm_prefix": arm_prefix,
            "enabled": use_gazebo_camera,
            "use_prefixed_frames": "true",
        }.items(),
    )
    actions.append(camera_launch)

    for namespace, xyz in zip(namespaces, poses):
        arm_description = _prefixed_urdf(robot_description, namespace, xyz)
        arm_robot_description = {"robot_description": arm_description}
        arm_semantic = {
            "robot_description_semantic": _prefixed_srdf(
                robot_description_semantic, namespace
            )
        }
        arm_joint_limits = _prefixed_joint_limits(moveit_config.joint_limits, namespace)
        arm_kinematics = _prefixed_kinematics(
            moveit_config.robot_description_kinematics
        )
        controller_config_path = _controller_config_path(namespace)

        actions.extend(
            [
                launch_ros.actions.Node(
                    package="custom_servo_demo",
                    executable="robot_description_republisher",
                    namespace=namespace,
                    name="robot_description_republisher",
                    parameters=[
                        arm_robot_description,
                        {
                            "topic_name": "robot_description",
                            "publish_period": 0.0,
                        },
                    ],
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    namespace=namespace,
                    parameters=[controller_config_path],
                    remappings=[
                        (
                            f"/{namespace}/controller_manager/robot_description",
                            "robot_description",
                        ),
                        (f"/{namespace}/joint_states", "/joint_states"),
                    ],
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager-timeout",
                        "300",
                        "--controller-manager",
                        f"/{namespace}/controller_manager",
                    ],
                ),
                launch_ros.actions.Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=namespace,
                    arguments=[
                        "panda_arm_controller",
                        "-c",
                        f"/{namespace}/controller_manager",
                    ],
                ),
                launch_ros.actions.Node(
                    package="moveit_servo",
                    executable="servo_node",
                    namespace=namespace,
                    name="servo_node",
                    parameters=[
                        _servo_params(base_servo_params, namespace),
                        acceleration_filter_update_period,
                        planning_group_name,
                        arm_robot_description,
                        arm_semantic,
                        arm_kinematics,
                        arm_joint_limits,
                    ],
                    output="screen",
                ),
                TimerAction(
                    period=3.0,
                    actions=[
                        launch_ros.actions.Node(
                            package="custom_servo_demo",
                            executable="servo_command_preparer",
                            namespace=namespace,
                            name="servo_command_preparer",
                            parameters=[
                                {
                                    "switch_command_type_service": (
                                        "servo_node/switch_command_type"
                                    ),
                                    "pause_servo_service": "servo_node/pause_servo",
                                    "command_type": 1,
                                    "timeout": 30.0,
                                }
                            ],
                            condition=IfCondition(prepare_servo),
                            output="screen",
                        )
                    ],
                ),
                launch_ros.actions.Node(
                    package="custom_servo_demo",
                    executable="joy_to_servo_twist",
                    namespace=namespace,
                    name="joy_to_servo_twist",
                    parameters=[
                        {
                            "joy_topic": "joy",
                            "twist_topic": "servo_node/delta_twist_cmds",
                            "frame_id": f"{namespace}_panda_link0",
                        }
                    ],
                    condition=IfCondition(use_joy_teleop),
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="manipulator_actions",
                    executable="pose_stamped_control",
                    namespace=namespace,
                    name="pose_stamped_end_effector_control",
                    parameters=[
                        {
                            "base_frame": f"{namespace}_panda_link0",
                            "ee_frame": f"{namespace}_panda_link8",
                            "twist_topic": "servo_node/delta_twist_cmds",
                            "pause_servo_service": "servo_node/pause_servo",
                            "switch_command_type_service": (
                                "servo_node/switch_command_type"
                            ),
                            "absolute_pose_topic": "end_effector_pose_absolute",
                            "relative_pose_topic": "end_effector_pose_relative",
                        }
                    ],
                    condition=IfCondition(use_pose_stamped_control),
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="manipulator_actions",
                    executable="action_server",
                    namespace=namespace,
                    name="manipulator_action_server",
                    parameters=[
                        {
                            "base_frame": f"{namespace}_panda_link0",
                            "ee_frame": f"{namespace}_panda_link8",
                            "twist_topic": "servo_node/delta_twist_cmds",
                            "pause_servo_service": "servo_node/pause_servo",
                            "switch_command_type_service": (
                                "servo_node/switch_command_type"
                            ),
                        }
                    ],
                    condition=IfCondition(launch_action_servers),
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="manipulator_actions",
                    executable="behavior_tree_runtime_server",
                    namespace=namespace,
                    name="behavior_tree_runtime_server",
                    condition=IfCondition(launch_action_servers),
                    output="screen",
                ),
            ]
        )
        if launch_behavior_tree:
            actions.append(
                launch_ros.actions.Node(
                    package="manipulator_actions",
                    executable="py_trees_runner",
                    namespace=namespace,
                    name="manipulator_py_trees_runner",
                    arguments=[
                        behavior_tree_name,
                        "--timeout",
                        behavior_tree_timeout,
                    ],
                    output="screen",
                )
            )

    return actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_count",
                default_value="1",
                description="Number of prefixed Panda arms to launch in one scene",
            ),
            DeclareLaunchArgument(
                "arm_prefix",
                default_value="arm",
                description="Prefix used to create namespaces such as arm_1 and arm_2",
            ),
            DeclareLaunchArgument(
                "arm_spacing",
                default_value="0.9",
                description="Distance in meters between adjacent arm bases on the Y axis",
            ),
            DeclareLaunchArgument(
                "use_joy_teleop",
                default_value="true",
                description="Launch a Joy-to-Servo bridge in each arm namespace",
            ),
            DeclareLaunchArgument(
                "use_pose_stamped_control",
                default_value="true",
                description="Launch a PoseStamped end-effector bridge in each arm namespace",
            ),
            DeclareLaunchArgument(
                "launch_action_servers",
                default_value="true",
                description="Launch one manipulator action server per arm namespace",
            ),
            DeclareLaunchArgument(
                "prepare_servo",
                default_value="true",
                description="Select Twist command mode and unpause each Servo node",
            ),
            DeclareLaunchArgument(
                "behavior_tree_name",
                default_value="none",
                description="Optional behavior tree name to run once per arm namespace",
            ),
            DeclareLaunchArgument(
                "behavior_tree_timeout",
                default_value="60.0",
                description="Timeout in seconds for optional behavior tree runners",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the aggregate multi-arm robot model",
            ),
            DeclareLaunchArgument(
                "use_gazebo_camera",
                default_value="true",
                description="Launch the single Gazebo camera/world process",
            ),
            OpaqueFunction(function=_arm_actions),
        ]
    )
