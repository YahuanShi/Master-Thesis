# morpheus_control

Rover drive control: Python control node and C++ ros2_control controller plugin.

## Components

### morpheus_control.py (Python)

Legacy control node that subscribes to `/cmd_vel` and `/joy`, computes four-wheel steering kinematics, and publishes to ForwardCommandControllers. Also handles joystick-based robotic arm joint control.

### kinematics.py

Pure kinematics module with three drive modes:
- **Ackermann** — forward/backward with turning radius
- **Pivot** — in-place rotation (all wheels angled to center)
- **Crab** — lateral translation (all wheels parallel)

### MorpheusDriveController (C++)

A `ros2_control` `ControllerInterface` plugin that directly claims steering position and wheel velocity hardware interfaces. Features:
- Same kinematics as the Python module
- Publishes `/wheel_odom` (nav_msgs/Odometry) from encoder integration
- Dynamic reconfigure via `ros2 param set` (wheel_base, drive_gain, deadzone, etc.)
- Registered as `morpheus_control/MorpheusDriveController` in pluginlib

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `wheel_base` | 1.072 | Front-to-rear axle distance (m) |
| `wheel_radius` | 0.125 | Wheel radius (m) |
| `wheel_separation` | 0.615 | Left-to-right wheel distance (m) |
| `drive_gain` | 10.0 | cmd_vel to joint velocity scale factor |
| `deadzone` | 0.05 | Minimum cmd_vel magnitude to act |

## Tests

- `test_kinematics.py` — 28 pytest cases covering all drive modes and edge cases
- `test_control_node.py` — launch_testing integration test
