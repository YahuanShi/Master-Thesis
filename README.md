# Morpheus

Morpheus is a simulated planetary rover built on **ROS 2 (Humble)** and **Gazebo**, operating in a recreated **MarsYard** terrain (with an additional NVIDIA Isaac Sim variant). The rover combines a four-wheel independent-steering chassis with a 7-DOF robotic arm, and the repository covers the full stack from robot description and low-level control to perception and autonomous navigation.

This project is a personal sandbox for learning and practicing core robotics topics, in particular:

- **Motion planning & navigation** — Nav2-based autonomous navigation (AMCL localization, planner/controller/smoother/behavior servers) on a custom MarsYard occupancy map generated from a digital elevation model.
- **Perception & sensor fusion** — stereo camera (ZED), LiDAR (scan + point cloud), IMU, ArUco marker detection, and EKF-based state estimation fusing odometry and IMU data.
- **Control algorithms** — chassis kinematics for independent four-wheel steering (Ackermann mixed steering, in-place pivot turn, crab walk), `ros2_control` based joint controllers, and joystick teleoperation of both the chassis and the robotic arm.

> **Status**: work in progress — the project is not fully complete yet, and more documentation/details will be added over time.

## Repository Layout

```
Morpheus/
├── MarsYard2024/          # Terrain assets (mesh, materials, textures, USD configs) — Git LFS
├── morpheus_isaac.usd     # Isaac Sim scene variant of the robot
├── run_morpheus_all.sh    # Convenience script to launch the full stack
└── morpheus_ws/src/
    ├── morpheus_description/  # URDF/xacro robot model (chassis, suspension, arm, sensors)
    ├── morpheus_simulation/   # Gazebo world bring-up, robot spawning, ROS↔Gazebo bridges
    ├── morpheus_control/      # Chassis kinematics & joystick control (drive + arm modes)
    ├── morpheus_nav2/         # Nav2 bring-up, map generation, EKF, twist_mux, teleop config
    └── ros2_aruco/            # Vendored third-party package for ArUco marker detection
```

## Robot Overview

- **Chassis**: four independently steered/driven wheels (`steering_*` + `wheel_*`) on a strut/shaft suspension, ~27 kg.
- **Arm**: 7-DOF manipulator (`arm_base → shoulder → elbow → wrist_1 → wrist_2 → wrist_3 → gripper`), with a fixed drill end-effector option.
- **Sensors**: Ouster LiDAR, ZED 2i and ZED Mini stereo cameras, IMU, plus sample-box payload mounts.

## Getting Started

```bash
# Build the workspace
cd morpheus_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Launch the full stack (Gazebo + control + perception)
./run_morpheus_all.sh
```

Or launch individual pieces directly, e.g.:

```bash
ros2 launch morpheus_simulation morpheus_spawn.launch.py
ros2 launch morpheus_control morpheus_control.launch.py
ros2 launch morpheus_nav2 bringup_nav2.launch.py
```

## License

MIT — see [LICENSE](LICENSE). The vendored `ros2_aruco` package retains its original MIT license and authorship.
