# morpheus_nav2

Navigation, SLAM, sensor fusion, perception, and mission autonomy for the Morpheus rover.

## Launch

```bash
ros2 launch morpheus_nav2 bringup_nav2.launch.py mode:=localization
ros2 launch morpheus_nav2 bringup_nav2.launch.py mode:=mapping
```

## Nodes

| Node | Description |
|---|---|
| Nav2 stack | planner, controller, smoother, behavior, bt_navigator, waypoint_follower |
| map_server + AMCL | Localization mode |
| slam_toolbox | Mapping mode (online async SLAM) |
| rgbd_odometry | Visual odometry from rtabmap (feeds EKF) |
| ground_segmentation | Height-threshold point cloud segmentation |
| aruco_localization | Marker-based EKF drift correction |
| diagnostic_aggregator | Health monitoring |

## Custom Behavior Tree

`behavior_trees/morpheus_nav_to_pose.xml` replaces the default Nav2 BT:
- **ArUco photo pause** — when a marker is detected mid-navigation, the rover stops for 3 seconds
- **Escalating recovery** — Spin 90° → 180° → 360° + BackUp on consecutive failures

The `IsArucoDetected` C++ BT condition plugin subscribes to `/aruco_markers`.

## Sensor Fusion (EKF)

Three sources fused via `robot_localization` EKF:
1. `odom0` — Gazebo ground-truth odometry (`/gz/odom`)
2. `imu0` — IMU yaw + angular velocity + linear acceleration
3. `odom1` — Visual odometry (`/visual_odom`, differential mode)

## Costmap Sources

Both global and local costmaps use three observation sources:
- `scan3d` — Segmented obstacle point cloud
- `scan2d` — 2D LiDAR scan
- `depth_cam` — Depth camera point cloud

## Key Configs

| File | Purpose |
|---|---|
| `nav2_params.yaml` | Nav2 server parameters, costmap config |
| `ekf.yaml` | EKF sensor fusion configuration |
| `slam_toolbox.yaml` | Online SLAM parameters |
| `visual_odom.yaml` | rtabmap rgbd_odometry config |
| `aruco_marker_map.yaml` | Known marker world poses (7 markers) |
| `diagnostics.yaml` | Diagnostic aggregator categories |
| `patrol_waypoints.yaml` | Waypoint patrol mission config |
| `twist_mux.yaml` | cmd_vel priority mux config |

## Known Limitations

- `slam_toolbox` requires a patched `getOdomPose()` for Gazebo's single-threaded executor (fix applied in Docker build)
- ArUco localization correction weight (0.3) and thresholds need real-world tuning
- Visual odometry quality depends on scene texture; featureless terrain degrades accuracy
