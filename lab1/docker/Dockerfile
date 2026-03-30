ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE_DIR=/workspace

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ament-cmake-python \
    ros-${ROS_DISTRO}-example-interfaces \
    sudo \
    vim && \
    rm -rf /var/lib/apt/lists/*

WORKDIR ${WORKSPACE_DIR}

RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN rosdep update

# Auto-source ROS and workspace on shell login
RUN cat <<'EOF' >> /etc/bash.bashrc
if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [ -n "${WORKSPACE_DIR}" ] && [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
EOF

CMD ["/bin/bash"]
