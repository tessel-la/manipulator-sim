ARG ROS_DISTRO=jazzy
ARG UBUNTU_CODENAME=noble
FROM ros:${ROS_DISTRO}-ros-base-${UBUNTU_CODENAME}

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

RUN echo 'APT::Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    gdb \
    git \
    nano \
    openssh-client \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-yaml \
    python3-vcstool \
    ruby \
    ruby-dev \
    sudo \
    tmux \
    vim \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-geometric-shapes \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-gripper-controllers \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-launch-param-builder \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-hybrid-planning \
    ros-${ROS_DISTRO}-moveit-kinematics \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-plugins \
    ros-${ROS_DISTRO}-moveit-resources-panda-description \
    ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-ros-robot-interaction \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-runtime \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-moveit-task-constructor-core \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-xacro \
    && gem install tmuxinator \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=rosuser
ENV USERNAME=${USERNAME}

RUN if id -u ${USERNAME} >/dev/null 2>&1; then \
      true; \
    elif getent passwd ${USER_UID} >/dev/null; then \
      EXISTING_USER="$(getent passwd ${USER_UID} | cut -d: -f1)" && \
      usermod --login ${USERNAME} --home /home/${USERNAME} --move-home ${EXISTING_USER}; \
    else \
      if getent group ${USER_GID} >/dev/null; then \
        GROUP_NAME="$(getent group ${USER_GID} | cut -d: -f1)"; \
      else \
        groupadd --gid ${USER_GID} ${USERNAME} && GROUP_NAME="${USERNAME}"; \
      fi && \
      useradd --uid ${USER_UID} --gid ${GROUP_NAME} -m ${USERNAME} -s /bin/bash; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
    mkdir -p /home/${USERNAME}/.config/tmuxinator /home/${USERNAME}/moveit_ws/src && \
    chown -R ${USERNAME}:${USER_GID} /home/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/moveit_ws

COPY --chown=${USERNAME}:${USER_GID} custom_servo_demo /home/${USERNAME}/moveit_ws/src/custom_servo_demo
COPY --chown=${USERNAME}:${USER_GID} manipulator_action_interfaces /home/${USERNAME}/moveit_ws/src/manipulator_action_interfaces
COPY --chown=${USERNAME}:${USER_GID} manipulator_actions /home/${USERNAME}/moveit_ws/src/manipulator_actions

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --symlink-install \
      --packages-select custom_servo_demo manipulator_action_interfaces manipulator_actions \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source ~/moveit_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

COPY --chown=${USERNAME}:${USER_GID} cors_mesh_server.py /home/${USERNAME}/cors_mesh_server.py
COPY --chown=${USERNAME}:${USER_GID} docker/scripts/ros_entrypoint.sh /home/${USERNAME}/ros_entrypoint.sh
COPY --chown=${USERNAME}:${USER_GID} docker/scripts/start_simulation.sh /home/${USERNAME}/start_simulation.sh
RUN chmod +x \
    /home/${USERNAME}/cors_mesh_server.py \
    /home/${USERNAME}/ros_entrypoint.sh \
    /home/${USERNAME}/start_simulation.sh

ENV SHELL=/bin/bash
ENV EDITOR=vim
ENV MOVEIT_WS=/home/${USERNAME}/moveit_ws
ENV COLCON_PACKAGES="custom_servo_demo manipulator_action_interfaces manipulator_actions"

ENTRYPOINT ["/home/rosuser/ros_entrypoint.sh"]

EXPOSE 8000

CMD ["bash"]
