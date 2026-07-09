import os
import tempfile
import xml.etree.ElementTree as ET

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml


DEFAULT_SCENE_CONFIG = os.path.join(
    get_package_share_directory("custom_servo_demo"),
    "config",
    "pick_place_scene.yaml",
)

SIM_CAMERA_TOPIC_ROOT = "/pick_place_sim/cameras"


def _truthy(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _pose(raw_pose):
    values = list(raw_pose or [])
    values.extend([0.0] * (6 - len(values)))
    return " ".join(f"{float(value):.6g}" for value in values[:6])


def _color(raw_color, fallback):
    values = list(raw_color or fallback)
    values.extend(list(fallback)[len(values) :])
    return " ".join(f"{float(value):.6g}" for value in values[:4])


def _text(parent, tag, text):
    node = ET.SubElement(parent, tag)
    node.text = str(text)
    return node


def _add_material(visual, color):
    material = ET.SubElement(visual, "material")
    _text(material, "ambient", color)
    _text(material, "diffuse", color)


def _box_geometry(parent, size):
    geometry = ET.SubElement(parent, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", " ".join(f"{float(value):.6g}" for value in size))


def _cylinder_geometry(parent, radius, length):
    geometry = ET.SubElement(parent, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    _text(cylinder, "radius", f"{float(radius):.6g}")
    _text(cylinder, "length", f"{float(length):.6g}")


def _add_box_model(world, name, pose, size, color, static=True):
    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "true" if static else "false")
    _text(model, "pose", pose)
    link = ET.SubElement(model, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    _box_geometry(collision, size)
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    _box_geometry(visual, size)
    _add_material(visual, color)


def _add_cylinder_model(world, name, pose, radius, length, color):
    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "true")
    _text(model, "pose", pose)
    link = ET.SubElement(model, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    _cylinder_geometry(collision, radius, length)
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    _cylinder_geometry(visual, radius, length)
    _add_material(visual, color)


def _add_visual_box(link, name, pose, size, color):
    visual = ET.SubElement(link, "visual", {"name": name})
    _text(visual, "pose", pose)
    _box_geometry(visual, size)
    _add_material(visual, color)


def _add_charuco_board(world, board):
    name = str(board.get("name", "charuco_board"))
    squares_x = int(board.get("squares_x", 5))
    squares_y = int(board.get("squares_y", 7))
    square_size = float(board.get("square_size", 0.035))
    thickness = float(board.get("thickness", 0.006))
    width = squares_x * square_size
    height = squares_y * square_size

    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "true")
    _text(model, "pose", _pose(board.get("pose")))
    link = ET.SubElement(model, "link", {"name": "link"})
    base_collision = ET.SubElement(link, "collision", {"name": "collision"})
    _box_geometry(base_collision, [width, height, thickness])
    _add_visual_box(
        link,
        "white_base",
        "0 0 0 0 0 0",
        [width, height, thickness],
        "0.96 0.96 0.92 1",
    )

    black = "0.02 0.02 0.02 1"
    for row in range(squares_y):
        for column in range(squares_x):
            x = (column + 0.5) * square_size - width * 0.5
            y = (row + 0.5) * square_size - height * 0.5
            if (row + column) % 2 == 0:
                _add_visual_box(
                    link,
                    f"checker_{row}_{column}",
                    f"{x:.6g} {y:.6g} {thickness * 0.55:.6g} 0 0 0",
                    [square_size * 0.96, square_size * 0.96, thickness * 0.12],
                    black,
                )
            else:
                marker = square_size * 0.42
                _add_visual_box(
                    link,
                    f"marker_center_{row}_{column}",
                    f"{x:.6g} {y:.6g} {thickness * 0.62:.6g} 0 0 0",
                    [marker, marker, thickness * 0.13],
                    black,
                )
                bit = square_size * 0.16
                bit_offset = square_size * 0.28
                _add_visual_box(
                    link,
                    f"marker_bit_a_{row}_{column}",
                    f"{x - bit_offset:.6g} {y + bit_offset:.6g} {thickness * 0.64:.6g} 0 0 0",
                    [bit, bit, thickness * 0.14],
                    black,
                )


def _add_camera_model(world, name, topic, pose, width=320, height=240, rate=10, static=True):
    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "true" if static else "false")
    _text(model, "pose", pose)
    link = ET.SubElement(model, "link", {"name": "camera_link"})
    if not static:
        _text(link, "gravity", "false")
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", "0.05")
    inertia = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"):
        _text(inertia, key, "0.00001")
    body = ET.SubElement(link, "visual", {"name": "camera_body"})
    _box_geometry(body, [0.05, 0.035, 0.035])
    _add_material(body, "0.02 0.02 0.025 1")
    sensor = ET.SubElement(link, "sensor", {"name": "image_sensor", "type": "camera"})
    _text(sensor, "always_on", "true")
    _text(sensor, "update_rate", str(rate))
    _text(sensor, "topic", topic)
    camera = ET.SubElement(sensor, "camera")
    _text(camera, "horizontal_fov", "1.047")
    image = ET.SubElement(camera, "image")
    _text(image, "width", str(width))
    _text(image, "height", str(height))
    _text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _text(clip, "near", "0.03")
    _text(clip, "far", "10.0")


def _scene_gz_image_topic():
    return f"{SIM_CAMERA_TOPIC_ROOT}/scene/image_raw"


def _wrist_gz_image_topic(namespace):
    topic_name = namespace or "wrist"
    return f"{SIM_CAMERA_TOPIC_ROOT}/{topic_name}/wrist/image_raw"


def _generate_world(scene_config, arm_count, arm_prefix, launch_scene_camera):
    config_path = scene_config or DEFAULT_SCENE_CONFIG
    with open(config_path, "r", encoding="utf-8") as config_file:
        scene = yaml.safe_load(config_file) or {}

    sdf = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(sdf, "world", {"name": "default"})
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})
    sensors = ET.SubElement(
        world,
        "plugin",
        {"filename": "gz-sim-sensors-system", "name": "gz::sim::systems::Sensors"},
    )
    _text(sensors, "render_engine", "ogre2")
    _text(world, "gravity", "0 0 -9.8")
    physics = ET.SubElement(world, "physics", {"name": "low_rate_physics", "type": "ignored"})
    _text(physics, "max_step_size", "0.02")
    _text(physics, "real_time_factor", "1.0")
    _text(physics, "real_time_update_rate", "50")
    scene_node = ET.SubElement(world, "scene")
    _text(scene_node, "ambient", "0.55 0.55 0.58 1")
    _text(scene_node, "background", "0.06 0.07 0.08 1")
    light = ET.SubElement(world, "light", {"name": "key_light", "type": "directional"})
    _text(light, "pose", "-2 -3 5 0 0 0")
    _text(light, "diffuse", "0.95 0.92 0.85 1")
    _text(light, "specular", "0.25 0.25 0.25 1")
    _text(light, "direction", "0.45 0.65 -1")

    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    _text(ground, "static", "true")
    ground_link = ET.SubElement(ground, "link", {"name": "link"})
    ground_collision = ET.SubElement(ground_link, "collision", {"name": "collision"})
    ground_geometry = ET.SubElement(ground_collision, "geometry")
    ground_plane = ET.SubElement(ground_geometry, "plane")
    _text(ground_plane, "normal", "0 0 1")
    _text(ground_plane, "size", "4 4")
    ground_visual = ET.SubElement(ground_link, "visual", {"name": "visual"})
    ground_visual_geometry = ET.SubElement(ground_visual, "geometry")
    ground_visual_plane = ET.SubElement(ground_visual_geometry, "plane")
    _text(ground_visual_plane, "normal", "0 0 1")
    _text(ground_visual_plane, "size", "4 4")
    _add_material(ground_visual, "0.24 0.28 0.27 1")

    table = scene.get("table") or {}
    _add_box_model(
        world,
        str(table.get("name", "pick_place_table")),
        _pose(table.get("pose", [0.5, 0.0, 0.02, 0.0, 0.0, 0.0])),
        table.get("size", [0.9, 0.7, 0.04]),
        _color(table.get("color"), [0.42, 0.45, 0.43, 1.0]),
    )

    for cube in scene.get("cubes", []) or []:
        size = float(cube.get("size", 0.06))
        _add_box_model(
            world,
            str(cube.get("name", "pick_cube")),
            _pose(cube.get("pose")),
            [size, size, size],
            _color(cube.get("color"), [0.95, 0.08, 0.06, 1.0]),
        )
    for target in scene.get("place_targets", []) or []:
        _add_cylinder_model(
            world,
            str(target.get("name", "place_target")),
            _pose(target.get("pose")),
            float(target.get("radius", 0.09)),
            float(target.get("height", 0.012)),
            _color(target.get("color"), [0.08, 0.38, 0.95, 1.0]),
        )
    for board in scene.get("charuco_boards", []) or []:
        _add_charuco_board(world, board)

    if launch_scene_camera:
        _add_camera_model(
            world,
            "scene_camera",
            _scene_gz_image_topic(),
            "0.55 0 0.82 0 1.5708 0",
            width=640,
            height=480,
            rate=10,
        )

    if arm_prefix:
        for index in range(1, arm_count + 1):
            namespace = f"{arm_prefix}_{index}"
            _add_camera_model(
                world,
                f"wrist_camera_{namespace}",
                _wrist_gz_image_topic(namespace),
                f"0.55 {((index - 1) * 0.10):.6g} 0.45 0 0 0",
                static=False,
            )
    else:
        _add_camera_model(
            world,
            "wrist_camera",
            _wrist_gz_image_topic(None),
            "0.55 0 0.45 0 0 0",
            static=False,
        )

    world_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="pick_place_world_",
        suffix=".sdf",
        delete=False,
    )
    with world_file:
        ET.ElementTree(sdf).write(world_file, encoding="unicode", xml_declaration=True)
    return world_file.name, config_path


def _camera_actions(context):
    arm_count = int(LaunchConfiguration("arm_count").perform(context))
    if arm_count < 1:
        raise RuntimeError("arm_count must be at least 1")

    enabled = _truthy(LaunchConfiguration("enabled").perform(context))
    if not enabled:
        return []

    if arm_count > 3:
        raise RuntimeError("gazebo_wrist_camera.launch.py supports up to 3 cameras")

    arm_prefix = LaunchConfiguration("arm_prefix").perform(context).strip("/")
    if not arm_prefix and arm_count > 1:
        raise RuntimeError("arm_prefix must be set when arm_count is greater than 1")

    use_prefixed_frames = _truthy(LaunchConfiguration("use_prefixed_frames").perform(context))
    launch_scene_camera = _truthy(LaunchConfiguration("launch_scene_camera").perform(context))

    scene_config = LaunchConfiguration("scene_config").perform(context)
    world_file = LaunchConfiguration("world_file").perform(context)
    if world_file:
        world_path = world_file
        resolved_scene_config = scene_config or DEFAULT_SCENE_CONFIG
    else:
        world_path, resolved_scene_config = _generate_world(
            scene_config,
            arm_count,
            arm_prefix,
            launch_scene_camera,
        )

    scene_publisher = launch_ros.actions.Node(
        package="custom_servo_demo",
        executable="pick_place_scene_publisher",
        name="pick_place_scene_publisher",
        parameters=[
            {
                "scene_config": resolved_scene_config,
                "frame_id": "world",
            }
        ],
        output="screen",
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

    actions = [gz_sim, set_pose_bridge, scene_publisher]
    if launch_scene_camera:
        scene_gz_image_topic = _scene_gz_image_topic()
        scene_gz_info_topic = f"{scene_gz_image_topic}/camera_info"
        actions.extend(
            [
                launch_ros.actions.Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="scene_camera_image_bridge",
                    arguments=[
                        f"{scene_gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                        "--ros-args",
                        "-r",
                        f"{scene_gz_image_topic}:=/scene_camera/image_raw",
                    ],
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="scene_camera_info_bridge",
                    arguments=[
                        f"{scene_gz_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "--ros-args",
                        "-r",
                        f"{scene_gz_info_topic}:=/scene_camera/camera_info",
                    ],
                    output="screen",
                ),
            ]
        )
    camera_configs = []
    if arm_prefix:
        camera_configs = [
            {
                "namespace": f"{arm_prefix}_{index}",
                "image_topic": f"/{arm_prefix}_{index}/wrist_camera/image_raw",
                "gz_image_topic": _wrist_gz_image_topic(f"{arm_prefix}_{index}"),
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
                "gz_image_topic": _wrist_gz_image_topic(None),
                "camera_info_output": "/wrist_camera/camera_info",
                "entity_name": "wrist_camera",
                "source_frame": "panda_hand",
            }
        ]

    for camera in camera_configs:
        namespace = camera["namespace"]
        image_topic = camera["image_topic"]
        gz_image_topic = camera["gz_image_topic"]
        gz_camera_info_topic = f"{gz_image_topic}/camera_info"

        actions.extend(
            [
                launch_ros.actions.Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    namespace=namespace,
                    name="wrist_camera_image_bridge",
                    arguments=[
                        f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                        "--ros-args",
                        "-r",
                        f"{gz_image_topic}:={image_topic}",
                    ],
                    output="screen",
                ),
                launch_ros.actions.Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    namespace=namespace,
                    name="wrist_camera_info_bridge",
                    arguments=[
                        f"{gz_camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "--ros-args",
                        "-r",
                        f"{gz_camera_info_topic}:={camera['camera_info_output']}",
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
                            "offset_roll": float(
                                LaunchConfiguration("wrist_camera_offset_roll").perform(context)
                            ),
                            "offset_pitch": float(
                                LaunchConfiguration("wrist_camera_offset_pitch").perform(context)
                            ),
                            "offset_yaw": float(
                                LaunchConfiguration("wrist_camera_offset_yaw").perform(context)
                            ),
                            "look_at_frame": LaunchConfiguration(
                                "wrist_camera_look_at_frame"
                            ).perform(context),
                            "look_at_x": float(
                                LaunchConfiguration("wrist_camera_look_at_x").perform(context)
                            ),
                            "look_at_y": float(
                                LaunchConfiguration("wrist_camera_look_at_y").perform(context)
                            ),
                            "look_at_z": float(
                                LaunchConfiguration("wrist_camera_look_at_z").perform(context)
                            ),
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
            DeclareLaunchArgument(
                "scene_config",
                default_value=DEFAULT_SCENE_CONFIG,
                description="YAML pick-place scene used to generate the Gazebo world",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="",
                description="Optional prebuilt SDF world; empty generates one from scene_config",
            ),
            DeclareLaunchArgument(
                "launch_scene_camera",
                default_value="true",
                description="Bridge a static overhead camera on /scene_camera/image_raw",
            ),
            DeclareLaunchArgument(
                "wrist_camera_offset_roll",
                default_value="0.0",
                description="Wrist camera mount roll offset relative to the hand frame",
            ),
            DeclareLaunchArgument(
                "wrist_camera_offset_pitch",
                default_value="-1.5708",
                description="Wrist camera mount pitch offset relative to the hand frame",
            ),
            DeclareLaunchArgument(
                "wrist_camera_offset_yaw",
                default_value="0.0",
                description="Wrist camera mount yaw offset relative to the hand frame",
            ),
            DeclareLaunchArgument(
                "wrist_camera_look_at_frame",
                default_value="pick_place_table",
                description="Optional scene frame for the simulated wrist camera to look at",
            ),
            DeclareLaunchArgument(
                "wrist_camera_look_at_x",
                default_value="0.45",
                description="Fallback world X target for the simulated wrist camera",
            ),
            DeclareLaunchArgument(
                "wrist_camera_look_at_y",
                default_value="0.0",
                description="Fallback world Y target for the simulated wrist camera",
            ),
            DeclareLaunchArgument(
                "wrist_camera_look_at_z",
                default_value="0.04",
                description="Fallback world Z target for the simulated wrist camera",
            ),
            OpaqueFunction(function=_camera_actions),
        ]
    )
