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
        ros-humble-robot-localization \
        ros-humble-twist-mux \
        ros-humble-teleop-twist-joy \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joy \
        ros-humble-tf-transformations \
        python3-colcon-common-extensions \
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
    && echo "source /opt/ros/humble/setup.bash" >> "/home/$USERNAME/.bashrc"

USER $USERNAME
WORKDIR /workspace/Morpheus
