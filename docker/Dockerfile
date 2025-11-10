FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

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
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

# Non-root user matching the host UID/GID, so build artifacts written into
# the mounted workspace keep the host user's ownership.
ARG USERNAME=morpheus
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid "$USER_GID" "$USERNAME" \
    && useradd --uid "$USER_UID" --gid "$USER_GID" -m -s /bin/bash "$USERNAME" \
    && echo "source /opt/ros/humble/setup.bash" >> "/home/$USERNAME/.bashrc"

USER $USERNAME
WORKDIR /workspace/Morpheus
