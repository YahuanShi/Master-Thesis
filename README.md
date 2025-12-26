# Morpheus

Morpheus is a simulated planetary rover built with **ROS 2 Humble**, **Gazebo / GZ Sim**, **Nav2**, `ros2_control`, and a custom Mars Yard environment.

The project is a robotics learning and systems-integration showcase: it brings together robot modeling, simulation, rover kinematics, autonomous navigation, SLAM, perception, sensor fusion, manipulation, and mission-level autonomy in one ROS 2 workspace.

## What It Does

Morpheus simulates a Mars rover that can drive, localize, map, navigate through rough terrain, detect visual markers, follow patrol missions, and control a robotic arm.

The stack includes:

- A four-wheel independently steered rover chassis
- A 7-DOF robotic arm with gripper/drill tooling
- A custom Mars Yard Gazebo world with terrain, obstacles, and ArUco targets
- LiDAR, stereo cameras, IMU, odometry, and point cloud processing
- Nav2 autonomous navigation with localization, planning, control, recovery, and waypoint following
- Online SLAM for unknown-environment mapping
- EKF-based sensor fusion
- 3D costmaps using segmented point clouds
- Joystick teleoperation for both the rover base and arm
- ArUco marker detection and marker-based mission behaviors
- MoveIt2-style arm planning and Cartesian end-effector control
- Visual odometry / visual SLAM integration as an additional localization source
- Custom Nav2 behavior-tree logic for task-aware autonomy
- Diagnostics, monitoring, RViz visualization, and test coverage

## Why I Built It

Morpheus is a project for practicing the core skills needed to build mobile robot software:

- Modeling a robot in URDF/xacro
- Connecting Gazebo simulation with ROS 2 topics, TF, sensors, and controllers
- Implementing rover kinematics for Ackermann steering, pivot turns, and crab walking
- Using `ros2_control` for joint-level command interfaces
- Building a Nav2 navigation stack from map, localization, planner, controller, costmaps, and lifecycle nodes
- Using SLAM, visual odometry, IMU, and wheel/ground-truth odometry for state estimation
- Processing 2D and 3D perception data for obstacle avoidance
- Designing autonomous waypoint missions with perception-triggered behaviors
- Structuring a multi-package ROS 2 workspace with launch files, configs, tests, and Docker support

## Main Features

### Rover Simulation

The rover runs in Gazebo on a Mars Yard terrain model. The simulation includes the robot model, physics, sensors, controller interfaces, ROS-Gazebo bridges, and RViz visualization.

### Chassis Control

The base controller converts `/cmd_vel` into steering angles and wheel velocities for a four-wheel independently steered rover.

Supported drive modes include:

- Straight driving
- Ackermann-style turning
- In-place pivot turns
- Crab walking

### Navigation and Mapping

Morpheus uses Nav2 for autonomous navigation. It supports both map-based localization and online mapping:

- AMCL localization on a prebuilt occupancy map
- `slam_toolbox` online SLAM
- Nav2 planner, controller, smoother, behavior server, and BT navigator
- Waypoint following and patrol missions
- Static, 2D LiDAR, and 3D point-cloud costmap sources

### Perception

The simulated sensor stack includes:

- LiDAR scan and point cloud
- Stereo camera streams
- IMU
- Gazebo odometry
- ArUco marker detection
- Ground/obstacle point cloud segmentation
- Visual odometry / visual SLAM input for localization

### Sensor Fusion

State estimation is handled through an EKF that combines odometry, IMU, and visual localization sources into a consistent TF tree for Nav2.

### Mission Autonomy

The patrol mission system loads waypoints from YAML, sends goals to Nav2, waits at each target, checks for ArUco markers, and continues or recovers based on task state.

Custom behavior-tree logic adds task-aware autonomy such as stopping for marker inspection, retrying failed goals, and scanning when navigation fails.

### Manipulation

The rover includes a 7-DOF arm with joint-level teleoperation and motion-planning support for Cartesian end-effector goals, inverse kinematics, and obstacle-aware planning.

### Testing

The project includes tests for:

- Rover kinematics
- Control-node command output
- URDF/xacro validity
- Link and joint consistency
- Launch-level controller behavior

## Architecture

See [docs/architecture.md](docs/architecture.md) for the full ROS node topology, TF tree, and data flow diagrams.

## Repository Layout

```text
Morpheus/
├── docker/                     # Reproducible ROS 2 / Gazebo environment
├── docs/                       # Architecture diagrams and documentation
├── MarsYard2024/               # Mars Yard mesh, texture, and USD assets
├── run_morpheus_all.sh         # Full-stack launcher
└── morpheus_ws/src/
    ├── morpheus_description/   # URDF/xacro robot model and meshes
    ├── morpheus_simulation/    # Gazebo worlds, spawning, bridges
    ├── morpheus_control/       # Rover kinematics and ros2_control drive controller
    ├── morpheus_nav2/          # Nav2, SLAM, perception, missions, behavior trees
    ├── morpheus_moveit_config/ # MoveIt2 arm planning configuration
    └── ros2_aruco/             # ArUco marker detection package
```

## Quick Start

The recommended way to run the project is with Docker.

```bash
./docker/run.sh
```

Inside the container:

```bash
cd /workspace/Morpheus/morpheus_ws
colcon build --symlink-install
source install/setup.bash

cd /workspace/Morpheus
./run_morpheus_all.sh
```

Run with online SLAM:

```bash
./run_morpheus_all.sh mode:=mapping
```

Run with map-based localization:

```bash
./run_morpheus_all.sh mode:=localization
```

## Useful Commands

Launch the full stack:

```bash
./run_morpheus_all.sh
```

Launch Nav2 directly:

```bash
ros2 launch morpheus_nav2 bringup_nav2.launch.py mode:=localization
```

Run the patrol mission:

```bash
ros2 run morpheus_nav2 patrol_mission.py
```

Run tests:

```bash
cd morpheus_ws
colcon test --packages-select morpheus_control morpheus_description
colcon test-result --verbose
```

## Tech Stack

- ROS 2 Humble
- Gazebo / GZ Sim
- Nav2
- `ros2_control`
- `slam_toolbox`
- `robot_localization`
- MoveIt2
- RViz
- Python / rclpy
- URDF / xacro
- Docker

## License

MIT. See [LICENSE](LICENSE).
