# Manipulator Sim

Robotic Manipulator Planning, Execution and Visualization environment.

This repository provides a robotic arm simulation environment specifically configured and tested for the [Robo-Boy](https://github.com/tessel-la/robo-boy) project. It utilizes components and setup based on the Moveit Tutorials Humble package.


## Features

-   **Dockerized Environment**: Ensures a consistent and reproducible setup using ROS 2 Humble and the MoveIt Tutorials Package.
-   **Panda Robot**: Currently focused on the Franka Emika Panda robot simulation.
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

This command builds the Docker image tagged as `manipulator-sim-image` using the `Dockerfile`.

```bash
docker build -t manipulator-sim-image .
```

This process might take some time, especially on the first build, as it downloads the ROS 2 base image and installs all dependencies.

### 3. Run the Docker Container

Once the image is built, you can start the container using Docker Compose:

```bash
docker-compose up -d
```

This command will start the `manipulator_sim_service` container in detached mode (`-d`). The container will be running, but no specific ROS processes or `tmuxinator` will be started automatically. You will have a bash shell as the default entry point.

### 4. Enter the Container and Start the Simulation

To start your simulation environment, you first need to get a shell inside the running container:

```bash
docker exec -it manipulator_sim_cont bash
```

Once inside the container's bash shell, navigate to your workspace (if necessary, though the entrypoint script should source it) and start the `tmuxinator` session:

```bash
# The ros_entrypoint.sh should have already sourced your ROS and workspace setup.
# If you have a specific root directory for tmuxinator project, ensure you are in a relevant place or have it configured in the .yml file.
# For example, if your tmuxinator config expects to be in moveit_ws:
# cd ~/moveit_ws 
tmuxinator start manipulator_simulation_setup
```

This will launch `tmux` with the panes and commands defined in `simulation/manipulator_simulation_setup.yml`:
1.  **moveit_launch**: Runs `ros2 launch moveit_servo servo_example.launch.py`
2.  **start_servo**: Calls the `/servo_node/start_servo` service.
3.  **servo_keyboard_input**: Runs `ros2 run moveit2_tutorials servo_keyboard_input` for teleoperation.

Your terminal will now be attached to this tmux session.

### 5. Interacting with Tmux (Attaching/Detaching)

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

### 6. Stopping the Simulation Container

To stop the Docker Compose setup and the container (which also stops any running tmux sessions and ROS nodes within it):

```bash
docker-compose down
```

## Customization

-   **Tmuxinator Configuration**: Modify `simulation/manipulator_simulation_setup.yml` to change the commands launched, the layout, or add more panes/windows.
-   **Dockerfile**: Add or remove ROS packages or system dependencies in the `Dockerfile` and rebuild the image.
-   **Docker Compose**: Adjust container settings, volumes, or environment variables in `docker-compose.yml`.



