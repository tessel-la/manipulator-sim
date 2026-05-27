# Manipulator Sim

Robotic Manipulator Planning, Execution and Visualization environment.

This repository provides a robotic arm simulation environment specifically configured and tested for the [Robo-Boy](https://github.com/tessel-la/robo-boy) project. It uses ROS 2 Jazzy, MoveIt 2, and a Panda MoveIt Servo setup.


## Features

-   **Dockerized Environment**: Ensures a consistent and reproducible setup using ROS 2 Jazzy on Ubuntu Noble.
-   **Panda Robot**: Currently focused on the Franka Emika Panda robot simulation.
-   **Reusable Action Control**: Provides ROS2 actions, a CLI, and YAML sequences for absolute and relative end-effector moves.
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
├── manipulator_actions/       # Python action server, CLI, motion helpers, and YAML sequences.
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

This will launch `tmux` with the panes and commands defined in `simulation/manipulator_simulation_setup.yml`:
1.  **moveit_launch**: Runs `ros2 launch custom_servo_demo servo_example.launch.py`
2.  **prepare_servo**: Selects Twist command mode with `/servo_node/switch_command_type` and unpauses Servo with `/servo_node/pause_servo`.
3.  **servo_keyboard_input**: Runs `ros2 run moveit_servo servo_keyboard_input` for teleoperation.
4.  **manipulator_actions**: Runs `ros2 run manipulator_actions action_server`.
5.  **cors_mesh_server**: Serves installed ROS mesh assets from `/opt/ros/${ROS_DISTRO}/share` on port 8000.

Your terminal will now be attached to this tmux session.

### 5. Controlling the Robot with Actions

The action server exposes two ROS2 actions:

-   `/move_end_effector` using `manipulator_action_interfaces/action/MoveEndEffector`
-   `/run_sequence` using `manipulator_action_interfaces/action/RunSequence`

For `/move_end_effector`, `relative: false` means an absolute base-frame target and `relative: true` means an offset from the current end-effector pose.

Use the CLI for common commands:

```bash
ros2 run manipulator_actions manipulator_cli move absolute --x 0.45 --y 0.0 --z 0.35 --yaw 0.0
ros2 run manipulator_actions manipulator_cli move relative --up 0.05
ros2 run manipulator_actions manipulator_cli move relative --forward 0.10 --yaw 0.3
ros2 run manipulator_actions manipulator_cli sequence run demo_pick
```

Directions are expressed in the Panda base frame: `forward/back` map to X, `left/right` map to Y, and `up/down` map to Z. The `yaw` argument is an angle in radians around the base Z axis.

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
