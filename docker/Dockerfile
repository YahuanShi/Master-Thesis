FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# All Morpheus nodes run inside this single container — no cross-machine ROS
# communication is needed. This host has several down/leftover network
# interfaces (dead onboard NIC, second USB-Ethernet, stale docker0 bridge)
# that FastDDS enumerates for multicast discovery, producing
# "Network is unreachable" / "sequence size exceeds remaining buffer" warnings
# on every node. Restricting DDS to loopback avoids this entirely.
# Set as a container-wide ENV (rather than via .bashrc) so it applies
# regardless of how a process is launched — interactive shell, `docker exec`,
# or a non-interactive script — since .bashrc only loads for interactive shells.
ENV ROS_LOCALHOST_ONLY=1

# Simulation / navigation / control stack used by the Morpheus rover but not
# included in the base desktop-full image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-xacro \
        ros-humble-ros-gz \
        ros-humble-ign-ros2-control \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-twist-mux \
        ros-humble-teleop-twist-joy \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joy \
        ros-humble-tf-transformations \
        ros-humble-moveit \
        ros-humble-moveit-servo \
        ros-humble-rtabmap-ros \
        ros-humble-diagnostic-aggregator \
        python3-colcon-common-extensions \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

# Non-root user matching the host UID/GID, so build artifacts written into
# the mounted workspace keep the host user's ownership.
ARG USERNAME=morpheus
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid "$USER_GID" "$USERNAME" \
    # GPU rendering (Gazebo's ogre2/EGL) needs direct access to the NVIDIA
    # GPU's DRM render nodes (/dev/dri/renderD128, /dev/dri/card1 — both
    # belong to PCI 65:00.0, the RTX A6000). `--gpus all` passes the device
    # nodes through but not their owning groups (host: video=44, render=109);
    # without membership, libEGL gets "Permission denied" and the renderer
    # silently falls back to software rasterization (the cause of the stutter).
    && groupadd -f --gid 109 render \
    && useradd --uid "$USER_UID" --gid "$USER_GID" -G video,render -m -s /bin/bash "$USERNAME" \
    && echo "source /opt/ros/humble/setup.bash" >> "/home/$USERNAME/.bashrc" \
    && echo "[ -f /tmp/slam_toolbox_src/install/setup.bash ] && source /tmp/slam_toolbox_src/install/setup.bash" >> "/home/$USERNAME/.bashrc" \
    && echo "[ -f /workspace/Morpheus/morpheus_ws/install/setup.bash ] && source /workspace/Morpheus/morpheus_ws/install/setup.bash" >> "/home/$USERNAME/.bashrc"

# Build slam_toolbox from source with a patched getOdomPose().
# The stock implementation uses tf_->transform() with the exact scan timestamp
# and zero timeout.  In Gazebo sim the single-threaded executor prevents /tf
# callbacks from running while the scan callback is active, so the TF buffer
# never contains the required stamp — every scan silently fails.  A blocking
# timeout deadlocks for the same reason.  The fix: lookupTransform with
# TimePointZero (latest available transform, non-blocking).
COPY slam_toolbox_tf_fix.patch /tmp/
RUN bash -c '\
    source /opt/ros/humble/setup.bash \
    && git clone --depth 1 -b humble https://github.com/SteveMacenski/slam_toolbox.git /tmp/slam_toolbox_src \
    && cd /tmp/slam_toolbox_src \
    && git apply /tmp/slam_toolbox_tf_fix.patch \
    && colcon build --packages-select slam_toolbox --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log src /tmp/slam_toolbox_tf_fix.patch \
    '

# Switch to CycloneDDS — FastDDS 2.6.x (Humble) cannot fully suppress its
# SHM discovery ports (/dev/shm/fastrtps_port*) via XML profile alone.
# With ipc:host the host and container share /dev/shm, so stale SHM segments
# from earlier runs block port init for later-starting nodes, causing them to
# fall through to different transports and become invisible to each other.
# CycloneDDS uses plain UDP multicast/unicast with no shared-memory segments.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY --chmod=755 entrypoint.sh /ros_entrypoint.sh

USER $USERNAME
WORKDIR /workspace/Morpheus

# Dev tools installed to user home — keeps system Python (ament) untouched
RUN pip3 install --user --no-cache-dir \
        ruff \
        pre-commit \
        transforms3d \
        vcstool
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
