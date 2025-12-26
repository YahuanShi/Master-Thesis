# morpheus_description

URDF/xacro model of the Morpheus rover.

## Structure

- `urdf/robot.xacro` — Top-level xacro that includes all sub-files
- `urdf/01_robot_description/` — Link and joint definitions, meshes
- `urdf/02_gazebo/gazebo.xacro` — Gazebo plugins, sensors (LiDAR, rgbd_camera, IMU)
- `urdf/03_ros2_control/` — Hardware interface definitions and controller configs

## Key Design Decisions

- **chassis_link** is the root frame (not base_link) to match the CAD origin. A static TF alias `chassis_link → base_link` is published for compatibility.
- **zed_2i** uses `rgbd_camera` sensor type (not plain `camera`) to provide RGB, depth, and point cloud from a single sensor.
- Resolution reduced to 640x360 for simulation performance.
- Separate `<camera_info_topic>` per camera to avoid CameraInfo collision between zed_2i and zed_mini.

## Controllers (morpheus_control.yaml)

| Controller | Type | Joints |
|---|---|---|
| forward_velocity_controller | ForwardCommandController | wheel_fl/fr/rl/rr (velocity) |
| forward_position_controller | ForwardCommandController | steering_fl/fr/rl/rr (position) |
| morpheus_drive_controller | MorpheusDriveController | All steering + wheel joints |
| arm_trajectory_controller | JointTrajectoryController | 6 arm joints |
| gripper_trajectory_controller | JointTrajectoryController | gripper_left_joint |
| robotic_arm_controller | ForwardCommandController | 7 arm+gripper joints |
