FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

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
    python3-vcstool \
    ruby \
    ruby-dev \
    sudo \
    tmux \
    vim \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-manager \
    ros-humble-geometric-shapes \
    ros-humble-geometry-msgs \
    ros-humble-gripper-controllers \
    ros-humble-interactive-markers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-trajectory-controller \
    ros-humble-joy \
    ros-humble-launch-param-builder \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-core \
    ros-humble-moveit-hybrid-planning \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-msgs \
    ros-humble-moveit-planners \
    ros-humble-moveit-plugins \
    ros-humble-moveit-resources-panda-description \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-robot-interaction \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-runtime \
    ros-humble-moveit-servo \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-visual-tools \
    ros-humble-pluginlib \
    ros-humble-robot-state-publisher \
    ros-humble-ros-base \
    ros-humble-ros2-control \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz2 \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-trajectory-msgs \
    ros-humble-xacro \
    && gem install tmuxinator \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=rosuser
ENV USERNAME=${USERNAME}

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
    mkdir -p /home/${USERNAME}/.config/tmuxinator /home/${USERNAME}/moveit_ws/src && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/moveit_ws

RUN git clone --depth 1 --branch humble https://github.com/ros-planning/moveit2_tutorials src/moveit2_tutorials

COPY --chown=${USERNAME}:${USERNAME} custom_servo_demo /home/${USERNAME}/moveit_ws/src/custom_servo_demo

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --symlink-install \
      --packages-select moveit2_tutorials custom_servo_demo \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source ~/moveit_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

COPY --chown=${USERNAME}:${USERNAME} cors_mesh_server.py /home/${USERNAME}/cors_mesh_server.py
RUN chmod +x /home/${USERNAME}/cors_mesh_server.py

RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    '' \
    'source /opt/ros/$ROS_DISTRO/setup.bash' \
    '' \
    'if [ -f /home/$USERNAME/moveit_ws/install/setup.bash ]; then' \
    '  source /home/$USERNAME/moveit_ws/install/setup.bash' \
    'fi' \
    '' \
    'exec "$@"' \
    > /home/${USERNAME}/ros_entrypoint.sh && \
    chmod +x /home/${USERNAME}/ros_entrypoint.sh

ENV SHELL=/bin/bash
ENV EDITOR=vim

ENTRYPOINT ["/home/rosuser/ros_entrypoint.sh"]

EXPOSE 8000

CMD ["bash"]
