ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000
ARG DEBIAN_FRONTEND=noninteractive

ENV WORKSPACE=/ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    cmake \
    python3-pip \
    python3-dev \
    python3-venv \
    gdb \
    nano \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG dialout,video ${USERNAME}

RUN mkdir -p ${WORKSPACE}/src ${WORKSPACE}/docs && \
    chown -R ${USER_UID}:${USER_GID} ${WORKSPACE} && \
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

COPY docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh && \
    chown ${USER_UID}:${USER_GID} /entrypoint.sh

USER ${USERNAME}
WORKDIR ${WORKSPACE}

RUN colcon build --symlink-install && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]