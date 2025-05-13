# Start with an official ROS 2 Humble base image
FROM ros:humble-ros-base

# Set environment variables to prevent interactive prompts and define ROS distro
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Install essential packages, ROS development tools, git, vcstool, and other utilities
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        cmake \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        sudo \
        vim \
        # For tmuxinator
        tmux \
        ruby \
        ruby-dev \
        # Essential MoveIt2 packages
        ros-humble-moveit \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-ros-planning-interface \
        ros-humble-moveit-ros-planning \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-planners \
        ros-humble-moveit-plugins \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-moveit-visual-tools \
        ros-humble-moveit-resources-panda-moveit-config \
        ros-humble-moveit-resources-panda-description \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-ros-perception \
        ros-humble-moveit-ros-robot-interaction \
        ros-humble-moveit-runtime \
        ros-humble-moveit-servo \
        ros-humble-moveit-setup-assistant \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-rviz2 \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install tmuxinator gem
RUN sudo gem install tmuxinator

# Create a non-root user 'rosuser'
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=rosuser
ENV USERNAME=$USERNAME
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    # Source ROS setup in .bashrc for interactive shells
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# Switch to the non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Create tmuxinator config directory as rosuser
RUN mkdir -p /home/$USERNAME/.config/tmuxinator

# Create ROS 2 workspace
RUN mkdir -p ~/moveit_ws/src
WORKDIR /home/$USERNAME/moveit_ws

# Clone MoveIt2 tutorials
RUN git clone --branch humble https://github.com/ros-planning/moveit2_tutorials src/moveit2_tutorials

# Import MoveIt2 repositories
WORKDIR /home/$USERNAME/moveit_ws/src
RUN vcs import < moveit2_tutorials/moveit2_tutorials.repos

WORKDIR /home/$USERNAME/moveit_ws
# Install system dependencies for all packages in the workspace using rosdep
RUN sudo apt-get update && \
    (sudo rosdep init && echo "Rosdep initialized successfully." || echo "Rosdep already initialized or init failed, continuing...") && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --as-root apt:true && \
    # Clean up apt caches and ROS user-specific data
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    rm -rf /home/$USERNAME/.ros

# Install colcon mixin
RUN sudo apt install python3-colcon-common-extensions && \
    sudo apt install python3-colcon-mixin && \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default

# Build the workspace
WORKDIR /home/$USERNAME/moveit_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    export MAKEFLAGS="-j4" && \
    colcon build --symlink-install --executor sequential --mixin release

# Add sourcing of the new workspace to .bashrc for convenience in interactive shells
RUN echo "source ~/moveit_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Set up the entrypoint script
RUN echo '#!/bin/bash' > /home/$USERNAME/ros_entrypoint.sh && \
    echo 'set -e' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '# Source ROS 2 environment' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '# Source the workspace setup' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo 'if [ -f /home/$USERNAME/moveit_ws/install/setup.bash ]; then' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '  source /home/$USERNAME/moveit_ws/install/setup.bash' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo 'fi' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo '# Execute the command passed to the container' >> /home/$USERNAME/ros_entrypoint.sh && \
    echo 'exec "$@"' >> /home/$USERNAME/ros_entrypoint.sh

# Make the entrypoint script executable and set it as the entrypoint
RUN sudo chmod +x /home/$USERNAME/ros_entrypoint.sh
ENTRYPOINT ["/home/rosuser/ros_entrypoint.sh"]

# Default command to start a bash shell
CMD ["bash"] 