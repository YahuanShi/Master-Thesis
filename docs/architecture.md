# Morpheus Architecture

## ROS Node Topology

```text
                          ┌──────────────┐
                          │  Gazebo Sim  │
                          └──────┬───────┘
                                 │ gz bridges
          ┌──────────┬───────────┼──────────┬──────────┐
          ▼          ▼           ▼          ▼          ▼
      /clock     /scan      /camera_2i   /imu     /gz/odom
      /scan/points          /camera_2i/  (depth, points, info)
          │          │           │          │          │
          ▼          ▼           ▼          ▼          ▼
   ┌──────────┐ ┌────────┐ ┌─────────┐ ┌──────┐ ┌────────┐
   │ ground   │ │ aruco  │ │ rgbd    │ │      │ │        │
   │ segmen-  │ │ _node  │ │ _odom   │ │  EKF │◄┤        │
   │ tation   │ │        │ │(rtabmap)│ │      │ │        │
   └────┬─────┘ └───┬────┘ └────┬────┘ └──┬───┘ └────────┘
        │           │           │          │
        ▼           ▼           ▼          ▼
  /scan/points/  /aruco_    /visual_   odom→chassis_link (TF)
   obstacles     markers     odom
        │           │                      │
        ▼           ▼                      ▼
   ┌─────────────────────────────────────────────────────┐
   │                    Nav2 Stack                        │
   │  ┌──────────┐ ┌──────────┐ ┌─────────────────────┐ │
   │  │map_server│ │  AMCL /  │ │   bt_navigator      │ │
   │  │          │ │slam_tool │ │ (morpheus_nav_to_   │ │
   │  │          │ │  box     │ │  pose.xml)          │ │
   │  └──────────┘ └──────────┘ └─────────┬───────────┘ │
   │  ┌──────────┐ ┌──────────┐ ┌─────────▼───────────┐ │
   │  │ planner  │ │controller│ │  behavior_server    │ │
   │  │ _server  │ │ _server  │ │  (spin/wait/backup) │ │
   │  └──────────┘ └────┬─────┘ └─────────────────────┘ │
   │  ┌──────────┐      │      ┌──────────────────────┐ │
   │  │ smoother │      │      │ waypoint_follower    │ │
   │  └──────────┘      │      └──────────────────────┘ │
   │  ┌──────────────┐  │  ┌──────────────────────────┐ │
   │  │global_costmap│  │  │   local_costmap          │ │
   │  │(static+voxel │  │  │   (voxel+inflation)      │ │
   │  │ +inflation)  │  │  │                          │ │
   │  └──────────────┘  │  └──────────────────────────┘ │
   └────────────────────┼────────────────────────────────┘
                        │
                   /cmd_vel_nav
                        │
                        ▼
                  ┌───────────┐      ┌───────────────┐
                  │ twist_mux │◄─────│teleop_twist_  │
                  │           │      │joy (/cmd_vel_ │
                  └─────┬─────┘      │     joy)      │
                        │            └───────────────┘
                   /cmd_vel
                        │
                        ▼
              ┌───────────────────┐
              │ morpheus_control  │
              │ (Python node or   │
              │  MorpheusDrive    │
              │  Controller)      │
              └────────┬──────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
  /forward_pos    /forward_vel   /wheel_odom
  _controller     _controller
  /commands       /commands
          │            │
          ▼            ▼
   ┌──────────────────────────┐
   │   ros2_control           │
   │   (gz_ros2_control)      │
   │   joint_state_broadcaster│
   └──────────────────────────┘
```

## TF Tree

```text
map
 └── odom                        (EKF: odom→chassis_link)
      └── chassis_link           (robot base frame)
           ├── base_link         (static alias)
           ├── lidar_link        (LiDAR sensor)
           ├── zed_2i_link       (main stereo camera)
           ├── zed_mini_link     (arm camera)
           ├── imu_link          (IMU)
           ├── shaft_left_link
           │    └── strut_fl/rl  → steering_fl/rl → wheel_fl/rl
           ├── shaft_right_link
           │    └── strut_fr/rr  → steering_fr/rr → wheel_fr/rr
           └── arm_base_mount_link
                └── arm_base_link
                     └── shoulder_link
                          └── elbow_link
                               └── wrist_1_link
                                    └── wrist_2_link
                                         └── wrist_3_link
                                              └── gripper_link
```

## Data Flow

```text
Sensors → Bridges → Perception → State Estimation → Navigation → Control → Actuators

1. Gazebo sensors produce /scan, /imu, /camera_2i (rgb+depth+points), /gz/odom
2. ground_segmentation splits point cloud into ground / obstacles
3. rgbd_odometry (rtabmap) produces /visual_odom from stereo
4. EKF fuses /gz/odom + /imu + /visual_odom → publishes odom→chassis_link TF
5. AMCL (or slam_toolbox) produces map→odom TF
6. aruco_localization corrects EKF drift when markers are detected
7. Nav2 costmaps consume /scan + /scan/points/obstacles + /camera_2i/points
8. Nav2 planner+controller produce /cmd_vel_nav
9. twist_mux selects between Nav2 and joystick commands → /cmd_vel
10. morpheus_control (or MorpheusDriveController) converts /cmd_vel to joint commands
```
