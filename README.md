# Manipulator Sim

Robotic Manipulator Planning, Execution and Visualization environment.

This repository provides a robotic arm simulation environment specifically configured and tested for the [Robo-Boy](https://github.com/tessel-la/robo-boy) project. It uses ROS 2 Jazzy, MoveIt 2, and a Panda MoveIt Servo setup.


## Features

-   **Dockerized Environment**: Ensures a consistent and reproducible setup using ROS 2 Jazzy on Ubuntu Noble.
-   **Panda Robot**: Currently focused on the Franka Emika Panda robot simulation.
-   **Reusable Action Control**: Provides ROS2 actions, a CLI, YAML sequences, and basic behavior trees for absolute and relative end-effector moves.
-   **Automated Startup**: Uses `tmuxinator` and Docker Compose to automatically launch necessary ROS nodes in a structured tmux session upon container startup.

## Prerequisites

-   Docker ([Installation Guide](https://docs.docker.com/engine/install/))
-   Docker Compose ([Installation Guide](https://docs.docker.com/compose/install/))
-   An X11 server running on your host machine for GUI visualization (e.g., RViz). (Linux users usually have this by default. Windows users can use WSLg or VcXsrv. macOS users can use XQuartz.)

## Project Structure

```
.manipulator-sim/
├── Dockerfile                 # Defines the Docker image with ROS 2, MoveIt2, and dependencies.
├── docker-compose.yml         # Configures the Docker container services and automated startup.
├── custom_servo_demo/         # MoveIt Servo launch/config for the Panda simulation.
├── manipulator_action_interfaces/ # ROS2 action definitions for motion commands and sequences.
├── manipulator_actions/       # Python action server, CLI, motion helpers, YAML sequences, and behavior trees.
├── simulation/
│   └── manipulator_simulation_setup.yml # Tmuxinator config for launching ROS nodes.
└── README.md                  # This file.
```

## Setup and Usage

### 1. Clone the Repository (if you haven't already)

```bash
git clone git@github.com:tessel-la/manipulator-sim.git
cd manipulator-sim
```

### 2. Build the Docker Image

This command builds the Docker image and the ROS workspace packages copied into the image.

```bash
docker compose build
```

This process might take some time, especially on the first build, as it downloads the ROS 2 Jazzy base image, installs MoveIt 2 dependencies, and builds the workspace.

The Compose workspace cache volumes are named with a `jazzy` suffix so older Humble `install` setup files are not reused by accident.

### 3. Run the Docker Container

Once the image is built, you can start the container using Docker Compose:

```bash
docker compose up -d
```

This starts the `manipulator_sim_service` container in detached mode. On startup, the entrypoint sources ROS and rebuilds the mounted ROS workspace packages into Docker-managed `build`, `install`, and `log` volumes.

To disable the automatic workspace rebuild for faster container startup:

```bash
AUTO_BUILD_WORKSPACE=0 docker compose up -d
```

### 4. Enter the Container and Start the Simulation

To start your simulation environment, you first need to get a shell inside the running container:

```bash
docker exec -it manipulator_sim_cont bash
```

Once inside the container's bash shell, start the simulation helper:

```bash
/home/rosuser/start_simulation.sh
```

By default, this starts one arm as `/arm_1`. Spawn more arms through the same helper:

```bash
/home/rosuser/start_simulation.sh --arm-count 2
```

If the tmux session is already running, restart it so the new arm count is applied:

```bash
/home/rosuser/start_simulation.sh --arm-count 2 --restart
```

The Gazebo wrist-camera setup supports up to three arms. For higher counts, disable that pane:

```bash
/home/rosuser/start_simulation.sh --arm-count 4 --no-gazebo-camera --restart
```

The Gazebo pane now builds a camera-visible pick-place scene from
`custom_servo_demo/config/pick_place_scene.yaml`: colored cubes, a blue place
pad, a Charuco-style board, one wrist camera per arm, and a fixed overhead
`/scene_camera/image_raw` stream.

This will launch `tmux` with the panes and commands defined in `simulation/manipulator_simulation_setup.yml`:
1.  **multi_servo_launch**: Runs `ros2 launch custom_servo_demo multi_servo_example.launch.py` with `ARM_COUNT=1` and `ARM_PREFIX=arm` by default, creating `/arm_1`.
2.  **wrist_cameras**: Runs `ros2 launch custom_servo_demo gazebo_wrist_camera.launch.py` with the same arm count/prefix, mounting each Gazebo camera to its namespaced hand TF frame and exposing one wrist camera per arm namespace.
3.  **cors_mesh_server**: Serves installed ROS mesh assets from `/opt/ros/${ROS_DISTRO}/share` on port 8000.

Your terminal will now be attached to this tmux session.

You can also override other startup defaults through helper options or environment variables:

```bash
/home/rosuser/start_simulation.sh --arm-count 2 --arm-prefix robot --no-gazebo-camera
ARM_COUNT=2 ARM_PREFIX=robot USE_GAZEBO_CAMERA=false /home/rosuser/start_simulation.sh
```

Use another pick-place fixture layout without editing the launch files:

```bash
/home/rosuser/start_simulation.sh --scene-config /path/to/my_scene.yaml --restart
```

### 5. Controlling the Robot with Actions

The action server exposes modular ROS2 actions:

-   `/move_end_effector` using `manipulator_action_interfaces/action/MoveEndEffector`
-   `/run_sequence` using `manipulator_action_interfaces/action/RunSequence`
-   `/detect_object` using `manipulator_action_interfaces/action/DetectObject`
-   `/grasp_object` using `manipulator_action_interfaces/action/GraspObject`
-   `/place_object` using `manipulator_action_interfaces/action/PlaceObject`

For `/move_end_effector`, `relative: false` means an absolute base-frame target and `relative: true` means an offset from the current end-effector pose.
In the simulated pick-place scene, `/detect_object` resolves objects from `/pick_place_scene/objects` and TF frames such as `pick_cube_red`; the returned `PoseStamped` is in the requested frame or the arm base frame by default. `/grasp_object` and `/place_object` are simulation-ready action primitives: they detect the requested fixture, move through hover/approach/lift or hover/release/retreat poses, and keep simple held-object bookkeeping.

Example action calls in the first arm namespace:

```bash
ros2 action send_goal /arm_1/detect_object manipulator_action_interfaces/action/DetectObject "{query: red_cube, kind: cube}"
ros2 action send_goal /arm_1/grasp_object manipulator_action_interfaces/action/GraspObject "{object_id: red_cube}"
ros2 action send_goal /arm_1/place_object manipulator_action_interfaces/action/PlaceObject "{target_id: blue_place_pad}"
```

Use the CLI for common commands:

```bash
ros2 run manipulator_actions manipulator_cli move absolute --x 0.45 --y 0.0 --z 0.35 --yaw 0.0
ros2 run manipulator_actions manipulator_cli move relative --up 0.05
ros2 run manipulator_actions manipulator_cli move relative --forward 0.10 --yaw 0.3
ros2 run manipulator_actions manipulator_cli sequence run demo_pick
```

Directions are expressed in the Panda base frame: `forward/back` map to X, `left/right` map to Y, and `up/down` map to Z. The `yaw` argument is an angle in radians around the base Z axis.

### Behavior Trees

Basic behavior trees live in `manipulator_actions/config/trees/*.yaml` and currently run through `py_trees`. The YAML format is intentionally backend-neutral so the same tree descriptions can later be exported or adapted to BehaviorTree.CPP:

```yaml
name: demo_pick_tree
backend: py_trees
root:
  sequence:
    name: demo_pick_tree
    memory: true
    children:
      - move_relative:
          name: lift
          up: 0.05
      - wait:
          name: settle
          seconds: 0.5
      - move_absolute:
          name: return_home
          x: 0.45
          y: 0.0
          z: 0.35
          yaw: 0.0
```

Run a tree in the arm namespace so its action leaves resolve to that arm's action server:

```bash
ros2 run manipulator_actions py_trees_runner demo_pick_tree --ros-args -r __ns:=/arm_1
```

Existing sequence YAML files can also be run as simple sequence trees:

```bash
ros2 run manipulator_actions py_trees_runner demo_pick --ros-args -r __ns:=/arm_1
```

To inspect the current BehaviorTree.CPP-compatible XML skeleton:

```bash
ros2 run manipulator_actions py_trees_runner demo_pick_tree --export-btcpp-xml
```

The multi-arm launch can also start a tree runner automatically in each arm namespace when `behavior_tree_name` is set:

```bash
ros2 launch custom_servo_demo multi_servo_example.launch.py behavior_tree_name:=demo_pick_tree
BEHAVIOR_TREE_NAME=demo_pick_tree /home/rosuser/start_simulation.sh --restart
```

For the camera-integrated demo scene, run:

```bash
ros2 run manipulator_actions py_trees_runner camera_pick_place_demo --ros-args -r __ns:=/arm_1
```

That tree waits for `/arm_1/wrist_camera/image_raw`, detects the red cube, runs
the simulated grasp action, detects the blue place pad, then runs the simulated
place action.

### Joystick / `sensor_msgs/msg/Joy` Control

The Servo launch file can also relay `sensor_msgs/msg/Joy` input into the arm's MoveIt Servo twist topic:

```bash
ros2 launch custom_servo_demo servo_example.launch.py use_joy_teleop:=true
```

The default tmux setup already enables this bridge. It listens on `/joy` and publishes to `/servo_node/delta_twist_cmds` only while non-zero joystick input is active, then sends one zero command when the input is released.

Default axis mapping:

-   `axes[1]`: base-frame X velocity, forward/back
-   `axes[0]`: base-frame Y velocity, left/right
-   `axes[4]`: base-frame Z velocity, up/down
-   `axes[3]`: base-frame yaw velocity

To test with a synthetic Joy message:

```bash
ros2 topic pub -r 20 /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.5, 0.0, 0.0, 0.0], buttons: []}"
```

For a physical joystick, launch the ROS `joy_node` as well:

```bash
ros2 launch custom_servo_demo servo_example.launch.py use_joy_teleop:=true launch_joy_node:=true joy_dev:=/dev/input/js0
```

You can also run only the bridge after the simulation is up:

```bash
ros2 run custom_servo_demo joy_to_servo_twist --ros-args \
  -p joy_topic:=/joy \
  -p twist_topic:=/servo_node/delta_twist_cmds \
  -p enable_button:=4
```

Set `enable_button` to a button index if you want a deadman button; the default `-1` accepts Joy commands without a button press.

### PoseStamped End-Effector Control

The Servo launch file can also listen for `geometry_msgs/msg/PoseStamped` end-effector targets and drive them through the same closed-loop Servo controller used by the actions:

```bash
ros2 launch custom_servo_demo servo_example.launch.py use_pose_stamped_control:=true
```

The default tmux setup enables this bridge. It listens for absolute targets on `/end_effector_pose_absolute` and relative base-frame offsets on `/end_effector_pose_relative`. Absolute commands can use the robot base frame or another TF-resolvable `header.frame_id`, such as `world`. The pose position is interpreted as `x/y/z`; the pose orientation is reduced to yaw around the robot base Z axis.

To move to an absolute target:

```bash
ros2 topic pub --once /end_effector_pose_absolute geometry_msgs/msg/PoseStamped "{header: {frame_id: panda_link0}, pose: {position: {x: 0.45, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}"
```

To command a relative offset from the current end-effector pose:

```bash
ros2 topic pub --once /end_effector_pose_relative geometry_msgs/msg/PoseStamped "{header: {frame_id: panda_link0}, pose: {position: {x: 0.05, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

You can also run only the bridge after the simulation is up:

```bash
ros2 run manipulator_actions pose_stamped_control --ros-args \
  -p absolute_pose_topic:=/end_effector_pose_absolute \
  -p relative_pose_topic:=/end_effector_pose_relative \
  -p base_frame:=panda_link0 \
  -p ee_frame:=panda_link8
```

If you prefer a single topic, set `pose_topic` and choose its interpretation with `relative:=true` or `relative:=false`.

Do not publish these bridge commands to Servo's native `/servo_node/pose_target_cmds` topic. The bridge intentionally listens on `/end_effector_pose_absolute` and `/end_effector_pose_relative`, then publishes Servo twist commands internally.

### Multiple Arms In One Scene

You can launch multiple Panda arms in one shared RViz/TF scene. The launch generates one aggregate `/robot_description` with prefixed links and joints such as `arm_1_panda_link0`, `arm_2_panda_link0`, and `arm_3_panda_link0`, then keeps Servo, controller managers, Joy bridges, PoseStamped bridges, and action servers under per-arm namespaces:

```bash
/home/rosuser/start_simulation.sh --arm-count 3 --restart
```

For more than three arms, disable the Gazebo wrist-camera pane:

```bash
/home/rosuser/start_simulation.sh --arm-count 4 --no-gazebo-camera --restart
```

If you are launching ROS directly instead of using the helper, pass the same count as a launch argument:

```bash
ros2 launch custom_servo_demo multi_servo_example.launch.py arm_count:=3
```

Useful namespaced interfaces:

-   `/robot_description`
-   `/joint_states`
-   `/arm_1/robot_description`
-   `/arm_1/servo_node/delta_twist_cmds`
-   `/arm_1/servo_node/switch_command_type`
-   `/arm_1/servo_node/pause_servo`
-   `/arm_1/joy`
-   `/arm_1/end_effector_pose_absolute`
-   `/arm_1/end_effector_pose_relative`
-   `/arm_1/move_end_effector`
-   `/arm_1/run_sequence`
-   `/arm_1/detect_object`
-   `/arm_1/grasp_object`
-   `/arm_1/place_object`
-   `/arm_1/wrist_camera/image_raw`
-   `/arm_1/wrist_camera/camera_info`
-   `/scene_camera/image_raw`
-   `/scene_camera/camera_info`
-   `/pick_place_scene/objects`

For example, move the second arm with a synthetic Joy message:

```bash
ros2 topic pub -r 20 /arm_2/joy sensor_msgs/msg/Joy "{axes: [0.0, 0.5, 0.0, 0.0, 0.0], buttons: []}"
```

The launch accepts:

-   `arm_count`: number of arms in the shared scene, default `1`
-   `arm_prefix`: namespace prefix, default `arm`
-   `arm_spacing`: Y-axis distance between adjacent bases, default `0.9`
-   `use_joy_teleop`: launch one Joy bridge per arm, default `true`
-   `use_pose_stamped_control`: launch one PoseStamped bridge per arm, default `true`
-   `launch_action_servers`: launch one action server per arm, default `true`
-   `prepare_servo`: switch each Servo node to Twist mode and unpause it, default `true`
-   `behavior_tree_name`: optional behavior tree to run once per arm namespace, default empty
-   `behavior_tree_timeout`: optional behavior tree timeout in seconds, default `60.0`
-   `use_rviz`: launch RViz with the aggregate multi-arm robot model, default `true`
-   `use_gazebo_camera`: launch the single Gazebo camera/world process, default `true`
-   `scene_config`: YAML file defining cubes, place pads, and Charuco-style boards
-   `launch_scene_camera`: launch the fixed overhead camera, default `true`

The startup helper maps these launch arguments from options and environment variables:

-   `--arm-count` / `ARM_COUNT`: defaults to `1`
-   `--arm-prefix` / `ARM_PREFIX`: defaults to `arm`
-   `--no-gazebo-camera` / `USE_GAZEBO_CAMERA=false`: Gazebo wrist cameras default to enabled
-   `--scene-config` / `PICK_PLACE_SCENE_CONFIG`: override the default pick-place fixtures
-   `--no-scene-camera` / `LAUNCH_SCENE_CAMERA=false`: disable the fixed overhead camera
-   `WRIST_CAMERA_OFFSET_ROLL`, `WRIST_CAMERA_OFFSET_PITCH`, `WRIST_CAMERA_OFFSET_YAW`: tune the Gazebo wrist camera mount orientation; pitch defaults to `-1.5708` so the simulated optical axis is aimed down into the workspace from the hand
-   `WRIST_CAMERA_LOOK_AT_FRAME`: scene frame the simulated wrist camera points at while its position follows the hand, default `pick_place_table`
-   `USE_JOY_TELEOP`: defaults to `true`
-   `USE_POSE_STAMPED_CONTROL`: defaults to `true`
-   `LAUNCH_ACTION_SERVERS`: defaults to `true`
-   `PREPARE_SERVO`: defaults to `true`
-   `BEHAVIOR_TREE_NAME`: defaults to empty
-   `BEHAVIOR_TREE_TIMEOUT`: defaults to `60.0`

This is one RViz/ROS-control simulation scene with prefixed robot models and a shared global TF tree. The Gazebo process is the camera and fixture world: it generates objects from the scene YAML and publishes TF frames such as `pick_cube_red`, `place_target_blue`, and `charuco_board` for demos or future perception replacement.

Gazebo sensors publish on private transport topics under `/pick_place_sim/cameras/...`; the launch remaps only the intended bridges back to the public ROS topics above. This keeps older dummy camera worlds from racing the pick-place camera stream if they are accidentally left around.

Reusable routines live in `manipulator_actions/config/sequences/*.yaml`:

```yaml
steps:
  - move_relative:
      up: 0.05
  - wait:
      seconds: 0.5
  - move_absolute:
      x: 0.45
      y: 0.0
      z: 0.35
      yaw: 0.0
```

After editing mounted source files inside Docker, rebuild the workspace:

```bash
colcon build --symlink-install --packages-select custom_servo_demo manipulator_action_interfaces manipulator_actions
source install/setup.bash
```

The same package list is also used by the Docker entrypoint via the `COLCON_PACKAGES` environment variable in `docker-compose.yml`.

### 6. Interacting with Tmux (Attaching/Detaching)

**Navigating Tmux Panes:**

`tmux` uses a prefix key, by default `Ctrl+b`. After pressing the prefix, you press another key for an action:

-   `Ctrl+b` then release and click `Up Arrow`: Move to the pane above.
-   `Ctrl+b` then release and click `Down Arrow`: Move to the pane below.
-   `Ctrl+b` then release and click `Left Arrow`: Move to the pane to the left.
-   `Ctrl+b` then release and click `Right Arrow`: Move to the pane to the right.
-   `Ctrl+b` then release and click `o`: Cycle through panes.
-   `Ctrl+b` then release and click `%`: Split current pane vertically.
-   `Ctrl+b` then release and click `"`: Split current pane horizontally.
-   `Ctrl+b` then release and click `x`: Kill the current pane (prompts for confirmation).
-   `Ctrl+b` then release and click `d`: Detach from the current tmux session (leaves the session running in the background).

**Re-attaching to a Detached Tmux Session:**

If you detach from the tmux session (using `Ctrl+b` then `d`) or if your `docker exec` session ends but the container and tmux session are still running, you can re-attach:

1.  Get a shell inside the container again if you're not already in one:
    ```bash
    docker exec -it manipulator_sim_cont bash
    ```
2.  From within the container's shell, list existing tmux sessions:
    ```bash
    tmux ls
    ```
3.  Attach to your session (assuming it's named `manipulator_simulation_setup`):
    ```bash
    tmux attach-session -t manipulator_simulation_setup
    ```

You can find more `tmux` commands online (e.g., a `tmux` cheatsheet).

### 7. Stopping the Simulation Container

To stop the Docker Compose setup and the container (which also stops any running tmux sessions and ROS nodes within it):

```bash
docker compose down
```

## Customization

-   **Tmuxinator Configuration**: Modify `simulation/manipulator_simulation_setup.yml` to change the commands launched, the layout, or add more panes/windows.
-   **Dockerfile**: Add or remove ROS packages or system dependencies in the `Dockerfile` and rebuild the image.
-   **Docker Compose**: Adjust container settings, volumes, or environment variables in `docker-compose.yml`.
