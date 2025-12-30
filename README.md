# Morpheus

A simulated Mars rover built with **ROS 2 Humble** and **Gazebo**, featuring four-wheel independent steering, a 7-DOF robotic arm, autonomous navigation, SLAM, sensor fusion, perception, and mission-level autonomy.

<!-- TODO: Add demo GIF/video here -->
<!-- ![Morpheus Demo](docs/demo.gif) -->

## Highlights

- **Four-wheel independent steering** with Ackermann, pivot turn, and crab walk drive modes
- **Custom `ros2_control` controller** (C++) with real-time wheel odometry publishing
- **Nav2 autonomous navigation** with costmap, planner, controller, and recovery behaviors
- **Online SLAM** via patched `slam_toolbox` (custom TF fix for Gazebo single-threaded executor)
- **EKF sensor fusion** combining wheel odometry, IMU, visual odometry, and ArUco corrections
- **3D perception pipeline** with LiDAR point cloud ground segmentation feeding voxel costmaps
- **ArUco marker detection** driving localization correction and mission-aware behavior tree logic
- **7-DOF arm** with MoveIt2 planning, Cartesian control, and joystick teleoperation
- **Patrol mission system** with YAML waypoints, Nav2 goal tracking, and marker-triggered actions
- **Custom Mars Yard** environment modeled in Blender with realistic terrain and obstacles
- **CI pipeline** with GitHub Actions, Docker builds, and automated testing
- **57+ unit and integration tests** across kinematics, perception, URDF validation, BT plugins, and launch tests

## Architecture

```
                    ┌─────────────┐
                    │  Joystick   │
                    └──────┬──────┘
                           │ /joy
                    ┌──────▼──────┐         ┌──────────────┐
                    │  Teleop /   │ cmd_vel │  Nav2 Stack  │
                    │  Twist Mux  │◄────────│  (planner,   │
                    └──────┬──────┘         │  controller, │
                           │                │  BT, recovs) │
                    ┌──────▼──────┐         └──────▲───────┘
                    │  Drive Ctrl │                │
                    │ (ros2_ctrl) │         ┌──────┴───────┐
                    └──────┬──────┘         │   Costmaps   │
                           │                │   (2D + 3D)  │
                    ┌──────▼──────┐         └──────▲───────┘
                    │   Gazebo    │                │
                    │  Sim + HW   │──sensors──►  Perception
                    └─────────────┘             (LiDAR, cam,
                                                 ground seg,
                                                 ArUco, EKF)
```

Full node topology, TF tree, and data flow in [docs/architecture.md](docs/architecture.md).

## Repository Structure

```
Morpheus/
├── docker/                         Dockerfile, compose, GPU passthrough
├── docs/                           Architecture diagrams
├── MarsYard2024/                   Mars Yard mesh/texture/USD assets
├── .github/workflows/ci.yml        CI: build + test in Docker
├── run_morpheus_all.sh             One-command full-stack launcher
└── morpheus_ws/src/
    ├── morpheus_description/       URDF/xacro model, meshes, sensor macros
    ├── morpheus_simulation/        Gazebo worlds, spawning, ROS-GZ bridges
    ├── morpheus_control/           Kinematics, ros2_control drive controller
    ├── morpheus_nav2/              Nav2, SLAM, EKF, perception, missions, BTs
    ├── morpheus_moveit_config/     MoveIt2 arm planning + Cartesian demo
    └── ros2_aruco/                 ArUco marker detection (vendored)
```

## Quick Start

### Prerequisites

- Docker with NVIDIA Container Toolkit (for GPU-accelerated Gazebo rendering)
- A joystick/gamepad (optional, for teleoperation)

### One-Command Setup

```bash
git clone https://github.com/YahuanShi/Morpheus.git
cd Morpheus

# Build image, start container, enter shell (first run builds everything)
./docker/run.sh

# Inside the container — build the workspace (only needed once)
cd morpheus_ws && colcon build --symlink-install
```

All ROS 2 / Gazebo / Nav2 dependencies are installed in the container. The workspace is mounted from the host, so source edits and build artifacts persist across container restarts.

### Common Commands

```bash
# From the host — all commands go through docker/run.sh:
./docker/run.sh              # Enter container shell
./docker/run.sh build        # colcon build inside container
./docker/run.sh test         # Run all tests
./docker/run.sh launch       # Launch full simulation stack
./docker/run.sh stop         # Stop the container
./docker/run.sh clean        # Wipe build cache and rebuild image

# Inside the container:
./run_morpheus_all.sh                     # Default: map-based localization
./run_morpheus_all.sh mode:=mapping       # Online SLAM
ros2 run morpheus_nav2 patrol_mission.py  # Autonomous patrol
```

## Package Details

### morpheus_description

URDF/xacro robot model with parameterized sensor macros (LiDAR, stereo cameras, IMU), collision meshes, and `ros2_control` hardware interface declarations.

### morpheus_control

- Python kinematics library supporting Ackermann, pivot, and crab drive modes
- C++ `ros2_control` controller plugin with real-time command/state interfaces and wheel odometry
- Joystick-driven teleoperation for both the rover chassis and robotic arm
- Dynamic parameter reconfiguration for drive gain, deadzone, and wheel geometry

### morpheus_nav2

- Nav2 navigation stack with AMCL localization and `slam_toolbox` online SLAM
- EKF sensor fusion (`robot_localization`) combining wheel/visual odometry + IMU
- 3D LiDAR point cloud ground segmentation for voxel costmap obstacle detection
- ArUco marker-based localization correction with weighted pose averaging
- Custom behavior tree condition plugin (`IsArucoDetected`) for marker-aware autonomy
- Waypoint patrol mission manager with YAML-configurable routes
- DEM-to-occupancy-grid map generation for terrain-based planning

### morpheus_moveit_config

MoveIt2 configuration for the 7-DOF arm with SRDF, kinematics solver, planning pipeline, and a Cartesian end-effector control demo script.

### morpheus_simulation

Gazebo world definitions, robot spawning, and ROS-GZ bridge configuration for all sensor topics. Includes a custom Mars Yard world built from Blender-exported terrain.

## Docker

The entire development environment runs inside a single Docker container — no host-side ROS installation needed. `docker/run.sh` handles image building, container lifecycle, and workspace compilation.

- **Dockerfile**: Based on `osrf/ros:humble-desktop-full`, installs the full ROS 2 navigation/control/simulation stack plus dev tools (ruff, pre-commit, pytest), and builds a patched `slam_toolbox` from source
- **docker-compose.yml**: NVIDIA GPU passthrough, X11 forwarding, DRI devices, joystick input, host networking, and persistent build volumes
- **entrypoint.sh**: Auto-sources ROS 2, slam_toolbox, and workspace overlays — entering the container is immediately ready to `ros2 launch`
- **CI**: GitHub Actions runs build + test in the same ROS 2 container image

The `slam_toolbox` patch fixes a TF lookup deadlock specific to Gazebo's single-threaded executor — the stock `getOdomPose()` uses exact-timestamp lookups with zero timeout, which silently fails when the TF buffer can't update during the scan callback.

## Tech Stack

| Layer | Tools |
|-------|-------|
| Simulation | Gazebo (Ignition), ROS-GZ bridges |
| Robot Model | URDF, xacro, SDF |
| Control | ros2_control, ros2_controllers, custom C++ controller plugin |
| Navigation | Nav2, AMCL, slam_toolbox, robot_localization (EKF) |
| Perception | PCL ground segmentation, ArUco detection, visual odometry |
| Manipulation | MoveIt2, MoveIt Servo |
| Visualization | RViz2, diagnostic_aggregator |
| Infrastructure | Docker, GitHub Actions CI, colcon, pytest, gtest |

## License

MIT. See [LICENSE](LICENSE).
