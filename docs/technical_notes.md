# Morpheus Project — Technical Notes

Personal notes on every major concept and design decision encountered while
building this project, with the relevant code snippets for reference.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Simulation Environment — Ignition Gazebo](#2-simulation-environment)
3. [Robot Kinematics — Three Drive Modes](#3-robot-kinematics)
4. [Sensor Suite & ROS-Gazebo Bridges](#4-sensor-suite--ros-gazebo-bridges)
5. [State Estimation — EKF Sensor Fusion](#5-state-estimation--ekf-sensor-fusion)
6. [Localization — AMCL & the Static TF Bypass](#6-localization)
7. [Navigation Stack — Nav2](#7-navigation-stack--nav2)
8. [Costmap & Obstacle Representation](#8-costmap--obstacle-representation)
9. [Ground Segmentation & Slope Detection](#9-ground-segmentation--slope-detection)
10. [Path Planning — A* on an Occupancy Grid](#10-path-planning)
11. [Path Following — Regulated Pure Pursuit](#11-path-following--regulated-pure-pursuit)
12. [Behavior Tree Navigation](#12-behavior-tree-navigation)
13. [Patrol Mission](#13-patrol-mission)
14. [ArUco Marker Detection](#14-aruco-marker-detection)
15. [cmd_vel Routing — twist_mux](#15-cmd_vel-routing--twist_mux)
16. [ros2_control — Hardware Abstraction](#16-ros2_control--hardware-abstraction)
17. [DEM Map Generation](#17-dem-map-generation)
18. [Deployment — Docker & colcon](#18-deployment--docker--colcon)
19. [Key Design Decisions — Q&A](#19-key-design-decisions--qa)

---

## 1. System Overview

**What is Morpheus?**
A ROS 2 Mars rover simulation running on Ignition Gazebo Fortress.
The rover features:
- 4-wheel independent steering (Ackermann / pivot / crab)
- 6-DOF robotic arm with gripper
- 3D LiDAR + stereo cameras + IMU
- Autonomous patrol with Nav2, ArUco landmark detection, slope-aware costmaps

**Technology stack:**
```
ROS 2 Humble  ·  Ignition Gazebo Fortress  ·  Nav2  ·  robot_localization (EKF)
slam_toolbox  ·  ros2_control  ·  OpenCV (ArUco)  ·  Docker
```

**Full data flow (one sentence per stage):**
```
Gazebo sensors
  → ros_gz_bridge (topic conversion)
  → ground_segmentation (slope detection)
  → EKF (fuse odom + IMU → TF)
  → Nav2 costmap (mark obstacles)
  → A* planner (global path)
  → Regulated Pure Pursuit (cmd_vel)
  → twist_mux (priority merge with joystick)
  → morpheus_control (Twist → joint commands)
  → ros2_control (joint position / velocity)
  → Gazebo joints
```

---

## 2. Simulation Environment

### Ignition Gazebo Fortress (aka GZ Sim 6)

Ignition Gazebo replaced classic Gazebo in ROS 2.
Key difference: it uses a **plugin-based ECS** (Entity-Component-System) architecture
instead of Gazebo Classic's monolithic engine.

**World:** MarsYard 2022 — 50 × 50 m heightmap with up to 2 m elevation change.

**Launching Gazebo from a ROS 2 launch file:**
```python
# morpheus_spawn.launch.py
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([gz_pkg_path, '/gz_sim.launch.py']),
    launch_arguments={
        'gz_args': [world_arg, '.sdf', ' -v 1', ' -r'],  # -r: start running
        'gui': gui_arg,
    }.items(),
)
```

**Gazebo needs to find world files and meshes via an environment variable:**
```python
gazebo_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=[
        os.path.join(morpheus_simulation_path, 'worlds'),
        ':' + str(Path(morpheus_description_path).parent.resolve()),
    ],
)
```

**Spawning the robot:**
```python
# xacro is processed at Python import time → produces a URDF string
doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
robot_desc = doc.toprettyxml(indent='  ')

gz_spawn_entity = Node(
    package='ros_gz_sim', executable='create',
    arguments=['-string', robot_desc, '-x', spawn_x, '-y', spawn_y, '-z', spawn_z],
)
```

---

## 3. Robot Kinematics

**File:** `morpheus_control/scripts/kinematics.py`

The rover has 4 independently steered and driven wheels — this is called
**4-Wheel Independent Steering (4WIS)**. Three driving modes are supported:

### Mode 1 — Ackermann (arc turn while moving forward)

Used when both linear (`vx`) and angular (`wz`) velocity are commanded.
Each wheel steers to a different angle so all four wheel axes intersect
at a single instantaneous centre of rotation — preventing tyre scrub.

```python
def compute_drive(vx, vy, wz, params):
    if wz != 0.0:
        if vx != 0.0:
            _ackermann(vx, wz, params, pos, vel)  # forward + turn
        else:
            _pivot(wz, params, pos, vel)           # spin in place
    else:
        _crab(vx, vy, params, pos, vel)            # strafe / straight

def _ackermann(vx, wz, p, pos, vel):
    r = abs(vx) / wz * 2 * math.pi     # instantaneous turn radius
    r_bl = r + p.steering_track / 2.0  # left wheel has larger radius
    r_br = r - p.steering_track / 2.0  # right wheel has smaller radius

    a_fl = math.atan(p.wheel_base / r_bl)   # front-left steer angle
    a_fr = math.atan(p.wheel_base / r_br)   # front-right steer angle
    pos[0] = a_fl * 1.57   # scale to controller position units
    pos[1] = a_fr * 1.57

    # Left and right wheels travel at different speeds (proportional to their radius)
    vel[0] = sign * math.hypot(vx - wz * p.steering_track/2, wz * p.wheel_base/2)
    vel[1] = sign * math.hypot(vx + wz * p.steering_track/2, wz * p.wheel_base/2)
```

### Mode 2 — Pivot (in-place rotation)

Used when only `wz` is commanded (`vx == 0`).
All wheels steer in an X-pattern so each wheel is tangent to the rotation circle.
This minimises scrub during a zero-radius turn.

```python
def _pivot(wz, p, pos, vel):
    ang = math.atan(p.wheel_base / p.steering_track)  # ~34° for default chassis
    pos[0] = -ang; pos[1] = ang   # X-pattern: FL↙ FR↘
    pos[2] = ang;  pos[3] = -ang  #            RL↗ RR↖
    vel[0] = -wz;  vel[1] = wz    # left backward, right forward (CCW rotation)
    vel[2] = -wz;  vel[3] = wz
```

### Mode 3 — Crab (holonomic strafe)

Used when only `vx`/`vy` are commanded (`wz == 0`).
All four wheels point in the same direction (velocity vector direction).
Enables sideways movement — impossible with Ackermann-only steering.

```python
def _crab(vx, vy, p, pos, vel):
    angle = math.atan2(vy, vx)
    # Angles > ±90° are wrapped to avoid mechanically impossible steer positions
    if abs(angle) >= math.pi / 2:
        angle = -np.sign(angle) * (math.pi - abs(angle))
    pos[:] = angle           # all four wheels identical
    vel[:] = magnitude * sign * factor
```

---

## 4. Sensor Suite & ROS-Gazebo Bridges

Ignition Gazebo topics are in a different namespace than ROS 2 topics.
`ros_gz_bridge` translates between them at runtime.

**Bridge syntax:**
```
/ros_topic@ros_msg_type[gz_msg_type    → unidirectional GZ→ROS
/ros_topic@ros_msg_type]gz_msg_type    → unidirectional ROS→GZ
/ros_topic@ros_msg_type@gz_msg_type    → bidirectional
```

**Key bridges in `morpheus_spawn.launch.py`:**
```python
bridge_scan  = Node(executable='parameter_bridge',
    arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'])

bridge_cloud = Node(executable='parameter_bridge',
    arguments=['/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'])

bridge_imu   = Node(executable='parameter_bridge',
    arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'])

# Odom topic includes the world name — republish to a fixed name for EKF
bridge_odom  = Node(executable='parameter_bridge',
    arguments=['/world/marsyard2022/model/morpheus_rover/odometry@...'],
    remappings=[('.../odometry', '/gz/odom')])
```

**Sensor table:**

| Sensor | GZ Topic | ROS Topic | Used by |
|--------|----------|-----------|---------|
| 3D LiDAR | `/scan/points` | `/scan/points` | ground_segmentation → costmap |
| 2D LiDAR | `/scan` | `/scan` | costmap, AMCL |
| ZED 2i RGB | `/camera_2i/image` | `/camera_2i` | ArUco detection |
| ZED 2i Depth | `/camera_2i/depth_image` | same | (optional visual odom) |
| IMU | `/imu` | `/imu` | EKF |
| Gazebo Odom | world-scoped | `/gz/odom` | EKF |

---

## 5. State Estimation — EKF Sensor Fusion

**Package:** `robot_localization`
**File:** `morpheus_nav2/config/ekf.yaml`

The **Extended Kalman Filter** fuses multiple noisy sensor streams into
a single best-estimate odometry, published as the `odom → chassis_link` TF.

**Why EKF?**
- Wheel odometry alone drifts on slopes (wheel slip)
- IMU alone accumulates angular drift
- Combining them with a Kalman filter reduces error from both sources

**Fusion inputs in this project:**
```
/gz/odom       → position (x,y) + yaw  [Gazebo ground truth in simulation]
/imu           → angular velocity + linear acceleration
/visual_odom   → (optional) pose from camera-based SLAM
```

**EKF config structure (ekf.yaml):**
```yaml
ekf_node:
  ros__parameters:
    frequency: 30.0
    odom0: /gz/odom          # Gazebo ground-truth pose
    odom0_config: [true, true, false,   # x, y, z
                   false, false, true,  # roll, pitch, yaw
                   true, true, false,   # vx, vy, vz
                   false, false, true]  # vroll, vpitch, vyaw
    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  true, true, true,    # angular velocity
                  true, true, true]    # linear acceleration
```

**Result:** The EKF publishes the `odom → chassis_link` transform,
which is the primary odometry source for Nav2's costmap and controller.

---

## 6. Localization

### Normal approach: AMCL (Adaptive Monte Carlo Localization)

AMCL maintains a **particle filter** — a set of hypotheses about where the
robot is. Each particle is a (x, y, θ) pose guess. As the robot moves and
receives LiDAR scans, particles that match the scan better survive; others die.

```
particles  →  motion model updates them  →  scan matching weights them
           →  resample (high-weight survive)  →  publish map→odom TF
```

**Problem in this project:** On sloped terrain, the 3D LiDAR scan does not
match the flat 2D map. AMCL's particle filter diverges, publishing a wrong
`map→odom` transform that makes the rover plan paths in the wrong location.

### Solution: Static Identity TF Bypass

Since the EKF already uses Gazebo's ground-truth odometry, the `odom` frame
already coincides with the world (`map`) frame in simulation.
The `map→odom` transform is therefore the identity.

```python
# bringup_nav2.launch.py
static_map_odom = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    # translation=0,0,0  rotation=quaternion identity (0,0,0,1)
    arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    condition=IfCondition(is_not_mapping),
)
```

**AMCL still runs** (the lifecycle manager requires it) but is told not to
publish the TF:
```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    tf_broadcast: false   # ← key: static TF takes over
```

### TF Tree

```
map
 └── odom          ← static identity TF (published by static_transform_publisher)
      └── chassis_link   ← published by EKF
           ├── base_link (alias)
           ├── lidar_link
           └── ...
```

**Why this matters for Nav2:**
Nav2 costmaps and the path planner all work in the `map` frame.
If `map→odom` is wrong, the planned path appears shifted relative to
where the rover actually is — causing it to drive off the map.

---

## 7. Navigation Stack — Nav2

Nav2 is the ROS 2 navigation framework. All nodes use a **lifecycle**
(unconfigured → inactive → active) so they can be started/stopped in order.

**The lifecycle manager** drives this sequencing:
```yaml
lifecycle_manager:
  ros__parameters:
    node_names: [map_server, amcl, planner_server, controller_server,
                 smoother_server, behavior_server, bt_navigator, waypoint_follower]
    autostart: true
    bond_timeout: 0.0   # wait indefinitely (map can take 15+ s to load)
```

**Two modes:**

| Mode | Localisation | Map source |
|------|-------------|-----------|
| `localization` | AMCL | pre-built map.yaml |
| `mapping` | slam_toolbox | built online |

---

## 8. Costmap & Obstacle Representation

**Two costmaps run in parallel:**

| | Global costmap | Local costmap |
|--|---------------|--------------|
| Frame | `map` | `odom` (rolling window) |
| Size | 50 × 50 m (full world) | 10 × 10 m around rover |
| Update rate | 5 Hz | 10 Hz |
| Purpose | global path planning | real-time collision avoidance |

**Layer composition** (both costmaps use):
- `StaticLayer` — the pre-built map.png (global only)
- `VoxelLayer` — live 3D point cloud observations
- `InflationLayer` — inflates obstacle cells by a radius

**VoxelLayer input sources:**
```yaml
voxel_layer:
  observation_sources: scan3d scan2d depth_cam
  scan3d:                                    # 3D LiDAR (slope-filtered)
    topic: /scan/points/obstacles
    data_type: PointCloud2
  scan2d:                                    # 2D LiDAR
    topic: /scan
  depth_cam:                                 # stereo depth
    topic: /camera_2i/points
```

**Inflation:** The inflation layer adds a gradient of cost around each obstacle.
The Regulated Pure Pursuit controller uses this cost to slow down near obstacles.
```yaml
inflation_layer:
  inflation_radius: 0.7      # metres of clearance bubble
  cost_scaling_factor: 2.5   # how fast cost decays with distance
```

---

## 9. Ground Segmentation & Slope Detection

**File:** `morpheus_nav2/scripts/ground_segmentation.py`

Raw 3D LiDAR returns include ground, rocks, and slope faces.
Nav2's 2D costmap only understands "obstacle" or "free" — so we need to
convert 3D point clouds into meaningful 2D obstacle signals.

### Step 1: Height-based split

```python
ground_mask   = (z >= ground_min) & (z <= ground_max)    # z ∈ [-2, -0.15] m
obstacle_mask = (z >  ground_max) & (z <= obstacle_max)  # above rover body
```

### Step 2: Slope detection on ground points

Ground-level points on a slope should be treated as obstacles (rover can tip).
Algorithm: **grid-based Z-gradient**

```python
def detect_steep_ground(pts, cell_size=0.20, slope_threshold=0.10):
    # 1. Bin points into a 2D grid
    xi = ((x - x_min) / cell_size).astype(np.int32)
    yi = ((y - y_min) / cell_size).astype(np.int32)

    # 2. Compute mean Z per cell using numpy scatter (fast, no Python loops)
    #    np.add.at handles duplicate indices correctly (unlike z_sum[xi,yi] += z)
    np.add.at(z_sum, (xi, yi), z)
    np.add.at(z_cnt, (xi, yi), 1)
    z_mean = z_sum / z_cnt

    # 3. Compare each cell to 4 axis-aligned neighbours
    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
        diff = np.abs(z_mean[...] - z_mean_shifted[...])
        steep[...] |= (diff > slope_threshold)   # OR across all 4 directions

    return steep[xi, yi]   # map cell flag back to individual points
```

**slope_threshold / cell_size = tan(angle):**
`0.10 / 0.20 = 0.5 → ~27°` — terrain steeper than this is marked as obstacle.

### Step 3: Merge and publish

```python
all_obstacles = np.concatenate([pts[obstacle_mask], slope_pts])
self.obstacle_pub.publish(xyz_to_pointcloud2(all_obstacles, msg.header))
self.ground_pub.publish(xyz_to_pointcloud2(flat_pts, msg.header))
```

**Fast PointCloud2 parsing** (without Python-level iteration):
```python
# Read raw bytes with numpy stride tricks — ~10× faster than read_points()
data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
x = np.ndarray(len(data), dtype=np.float32, buffer=data,
                offset=x_off, strides=(msg.point_step,))
```

---

## 10. Path Planning

**Plugin:** `nav2_navfn_planner/NavfnPlanner` (alias: `GridBased`)

NavFn implements **A\*** search on the 2D occupancy costmap grid.

**A\* in brief:**
- Maintains an open list sorted by `f = g + h`
  - `g` = cost from start to current cell
  - `h` = heuristic (Euclidean distance to goal)
- Expands the lowest-f cell, adds unvisited neighbours
- Stops when the goal cell is reached

**Configuration:**
```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.3    # goal tolerance (metres)
      use_astar: true   # A* instead of Dijkstra
```

**Path smoothing** after planning:
```yaml
simple_smoother:
  plugin: "nav2_smoother::SimpleSmoother"
  w_smoothness: 3.0   # pull waypoints toward straight lines
  w_curvature: 0.5    # penalise sharp turns
```

---

## 11. Path Following — Regulated Pure Pursuit

**Plugin:** `nav2_regulated_pure_pursuit_controller`

Pure Pursuit computes a steering command to drive toward a **lookahead point**
on the planned path.

```
path:  ──────────────────●──────────────────► goal
                         ↑ lookahead point (L metres ahead)
rover: ●
steering curvature κ = 2 * lateral_error / L²
```

**"Regulated"** additions in this plugin:
1. **Curvature scaling**: slow down on tight curves
2. **Cost scaling**: slow down near obstacles (using costmap cost values)

```yaml
FollowPath:
  desired_linear_vel: 0.4       # m/s cruise speed
  lookahead_dist: 0.8           # metres to lookahead point
  use_velocity_scaled_lookahead_dist: true  # longer lookahead at higher speed
  cost_scaling_dist: 1.2        # start slowing when <1.2m from obstacle
  cost_scaling_gain: 0.5        # gentle slowdown curve
  regulated_linear_scaling_min_speed: 0.1  # minimum speed (never fully stops)
  rotate_to_heading: false       # continuous arc turns (no stop-and-spin)
```

**Progress checker** — detects if rover is stuck:
```yaml
progress_checker:
  required_movement_radius: 0.1  # must move 0.1 m within...
  movement_time_allowance: 8.0   # ...8 seconds or trigger recovery
```

---

## 12. Behavior Tree Navigation

**File:** `morpheus_nav2/behavior_trees/morpheus_nav_to_pose.xml`

Nav2 uses **Behavior Trees** (BT) to orchestrate the navigate-to-pose logic.
A BT is a tree of nodes that return SUCCESS, FAILURE, or RUNNING.

**Key BT node types:**
| Type | Behaviour |
|------|-----------|
| `Sequence` | Run children left-to-right; stop on first FAILURE |
| `ReactiveFallback` | Try children left-to-right; stop on first SUCCESS; re-evaluates every tick |
| `RecoveryNode` | Run main branch; if it fails, run recovery branch; retry N times |
| `RateController` | Only tick child at specified Hz |

**Morpheus BT structure:**
```xml
<RecoveryNode number_of_retries="3" name="NavigateRecovery">

  <!-- Main branch: replan at 5 Hz, follow path -->
  <PipelineSequence>
    <RateController hz="5.0">
      <ComputePathToPose goal="{goal}" planner_id="GridBased"/>
    </RateController>

    <!-- Reactive: if ArUco marker spotted, pause 3 s for "photograph" -->
    <ReactiveFallback>
      <Sequence name="ArucoPhotoSequence">
        <IsArucoDetected topic="/aruco_markers" timeout="1.0"/>
        <Wait wait_duration="3"/>
      </Sequence>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </ReactiveFallback>
  </PipelineSequence>

  <!-- Recovery: spin ±90° + clear costmaps, then wait 2 s -->
  <ReactiveFallback>
    <RoundRobin>
      <Sequence><ClearEntireCostmap/><Spin spin_dist="1.57"/></Sequence>
      <Sequence><Spin spin_dist="-1.57"/><ClearEntireCostmap/></Sequence>
      <Wait wait_duration="2"/>
    </RoundRobin>
  </ReactiveFallback>

</RecoveryNode>
```

**Why no Backup recovery?**
The rover cannot reverse on sloped Mars terrain — the rear of the chassis
would dig into the ground. Backup was removed; Spin is used instead.

---

## 13. Patrol Mission

**File:** `morpheus_nav2/scripts/patrol_mission.py`

Uses `nav2_simple_commander.BasicNavigator` — a Python API that wraps
the Nav2 action servers (NavigateToPose, etc.) into synchronous-style calls.

```python
class PatrolMission:
    def __init__(self):
        self.nav = BasicNavigator('patrol_mission')
        # Load waypoints from YAML config
        self.waypoints = self._load_waypoints(waypoints_file)

    def run(self):
        # 1. Set initial pose so AMCL particle filter starts near truth
        self.nav.setInitialPose(init_pose)

        # 2. Block until all Nav2 nodes are active (specifically waits for AMCL)
        self.nav.waitUntilNav2Active(localizer='amcl')

        # 3. Loop through waypoints
        for wp in self.waypoints:
            goal = self._make_goal(wp)    # PoseStamped in 'map' frame
            self.nav.goToPose(goal)       # sends NavigateToPose action goal

            # Per-waypoint timeout using monotonic time (not affected by sim time)
            deadline = time.monotonic() + self.waypoint_timeout
            while not self.nav.isTaskComplete():
                rclpy.spin_once(self.nav, timeout_sec=0.1)
                if time.monotonic() > deadline:
                    self.nav.cancelTask()   # skip unreachable waypoints
                    break

            if self.nav.getResult() == TaskResult.SUCCEEDED:
                self._dwell_and_detect(wp_name)   # pause + scan for ArUco

    def _dwell_and_detect(self, wp_name):
        # spin_once in loop (not time.sleep) so ArUco callbacks keep arriving
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.dwell_time:
            rclpy.spin_once(self.nav, timeout_sec=0.2)
```

**Waypoints (patrol_waypoints.yaml):**
```yaml
waypoints:
  - {name: north_east,  x:  3.0, y: 6.5, yaw: 1.57}
  - {name: east,        x:  4.0, y: 4.0, yaw: 0.0 }
  - {name: south_east,  x:  2.5, y: 2.0, yaw: -1.57}
  - {name: south,       x:  0.0, y: 1.5, yaw: 3.14}
  - {name: south_west,  x: -2.5, y: 2.0, yaw: 1.57}
  - {name: west,        x: -4.0, y: 4.0, yaw: 3.14}
  - {name: north_west,  x: -3.0, y: 6.5, yaw: 1.57}
  - {name: return_home, x:  0.0, y: 4.0, yaw: 0.0 }
```

---

## 14. ArUco Marker Detection

**Package:** `ros2_aruco`
**File:** `bringup_nav2.launch.py` → `aruco_localization.py`

ArUco markers are 2D binary-coded fiducials (like QR codes).
OpenCV detects them in a camera image and estimates their 6-DOF pose
relative to the camera.

**Detection node configuration:**
```python
aruco_node = Node(
    package='ros2_aruco', executable='aruco_node',
    parameters=[{
        'image_topic': '/camera_2i',
        'marker_size': 0.1,                       # 10 cm physical size
        'aruco_dictionary_id': 'DICT_5X5_50',     # 50 distinct 5×5 markers
    }]
)
```

**Published topic:** `/aruco_detector_2i/aruco_markers` → `ArucoMarkers.msg`
containing `marker_ids[]` and `poses[]`.

**In the BT:** `IsArucoDetected` is a custom BT condition node that returns
SUCCESS when a marker has been seen on this topic within the last `timeout` seconds.

**In the patrol mission:** Markers seen during dwell are logged:
```python
def _aruco_cb(self, msg):
    self.recent_markers = list(msg.marker_ids)
# → "ArUco markers detected at north_east: [3, 7]"
```

---

## 15. cmd_vel Routing — twist_mux

Multiple sources want to command the rover simultaneously:
- Nav2 autonomous navigation (`/cmd_vel_nav`)
- Joystick teleop (`/cmd_vel_joy`)
- (Future) emergency stop

**twist_mux** implements priority-based multiplexing:
the highest-priority active source wins, with a timeout.

```yaml
# twist_mux.yaml
topics:
  - name: joystick
    topic: /cmd_vel_joy
    timeout: 0.5     # if no message for 0.5 s, give up priority
    priority: 100    # higher = wins
  - name: navigation
    topic: /cmd_vel_nav
    timeout: 0.5
    priority: 10
```

**Why remap Nav2 output to `/cmd_vel_nav`?**
Both `controller_server` and `behavior_server` publish to `/cmd_vel` by default.
Remapping them prevents them from bypassing twist_mux:
```python
controller = Node(..., remappings=[('/cmd_vel', '/cmd_vel_nav')])
behavior   = Node(..., remappings=[('/cmd_vel', '/cmd_vel_nav')])
# twist_mux always runs → merges /cmd_vel_nav + /cmd_vel_joy → /cmd_vel
```

---

## 16. ros2_control — Hardware Abstraction

`ros2_control` separates **what to command** (controllers) from
**how to execute it** (hardware interface). In simulation, Gazebo acts as
the hardware interface via `gz_ros2_control`.

**Controller types used:**

| Controller | Purpose |
|-----------|---------|
| `joint_state_broadcaster` | Reads joint positions/velocities → `/joint_states` |
| `forward_position_controller` | Accepts target steering angles → commands them directly |
| `forward_velocity_controller` | Accepts target wheel speeds → commands them directly |
| `suspension_controller` | Manages passive suspension joints |
| `robotic_arm_controller` | 7-DOF arm position control |

**Spawning order is critical** (enforced via `OnProcessExit` event handlers):
```
gz_spawn_entity finishes
  → (1 s delay) → joint_state_broadcaster spawns
  → (on exit)   → velocity + position + suspension + arm controllers spawn
```

Without this order, controllers try to bind to joints before the robot
exists in Gazebo and immediately fail.

```python
# morpheus_spawn.launch.py
activate_jsb_after_spawn = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=gz_spawn_entity,
        on_exit=[TimerAction(period=1.0, actions=[spawner_jsb])],
    )
)
activate_rest_after_jsb = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=spawner_jsb,
        on_exit=[spawner_vel, spawner_pos, spawner_susp, spawner_arm],
    )
)
```

---

## 17. DEM Map Generation

**File:** `morpheus_nav2/scripts/gen_map_from_dem.py`

A 2D occupancy grid (`map.png` + `map.yaml`) is required by Nav2's
`map_server` and A\* planner. We generate it offline from the terrain's
Digital Elevation Model (DEM) image.

**Slope mode (default):**
Uses the **Sobel operator** — a finite-difference gradient filter.
High gradient pixels = steep terrain = obstacle.

```python
gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=5)   # X gradient
gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=5)   # Y gradient
mag = cv2.magnitude(gx, gy)                         # gradient magnitude ∝ slope
_, occ = cv2.threshold(mag, thresh, 255, cv2.THRESH_BINARY)
```

**Critical detail — negate=1 in map.yaml:**
```python
# Our image: white pixel = obstacle, black = free
# ROS map_server default: white = free, black = occupied  ← opposite!
# negate=1 flips the interpretation to match our image convention.
# Without negate=1, ~99% of the map appears occupied; Nav2 cannot plan any path.
write_map_yaml(map_yaml, 'map.png', resolution, origin, negate=1)
```

**Physical resolution from SDF:**
```python
sx, sy, sz = parse_heightmap_size(model_sdf)   # e.g. 50, 50, 2 (metres)
resolution = sx / float(image_width)            # metres per pixel
origin = (-sx/2, -sy/2, 0.0)                   # image bottom-left in world frame
```

---

## 18. Deployment — Docker & colcon

### colcon — ROS 2 build system

`colcon` builds all packages in `morpheus_ws/src` together,
resolving inter-package dependencies automatically.

```bash
cd morpheus_ws
colcon build --symlink-install
# --symlink-install: Python scripts are symlinked instead of copied
# → edit .py files without rebuilding; only CMake/C++ needs a rebuild
source install/setup.bash   # add built packages to ROS_PACKAGE_PATH
```

### Docker — reproducible environment

The entire ROS 2 + Gazebo + Nav2 stack runs inside a Docker container.
The host provides GPU passthrough (for Gazebo rendering) and X11 display.

```bash
# docker/run.sh — project entry point
./docker/run.sh          # start container + enter shell
./docker/run.sh build    # colcon build inside container
./docker/run.sh launch   # launch full simulation stack
./docker/run.sh test     # run unit tests
```

**Key docker-compose settings:**
- `runtime: nvidia` — NVIDIA GPU inside container
- `/dev/dri` volume — DRI device for OpenGL
- `/tmp/.X11-unix` volume + `DISPLAY` env — X11 forwarding for GUI
- Project directory mounted at `/workspace/Morpheus`

---


## 19. Engineering Notes — What I Learned Building This

Reflections on problems encountered during development: what the symptom was,
what I tried first, and what the actual root cause turned out to be.

---

### The rover ran off the map edge even though the map had walls

**Symptom:** After navigating for a few minutes, the rover's position in RViz
drifted far outside the actual terrain, and Nav2 planned a path straight off the edge.

**First attempt:** I assumed the waypoints were wrong and moved them further inward.
Didn't help — after a few laps the drift appeared again from a different direction.

**Root cause:** AMCL's particle filter was diverging on slopes. The 3D LiDAR
produces returns from the full terrain surface, but AMCL matches those returns
against a flat 2D occupancy map — on sloped ground the scan pattern looks nothing
like what the map predicts. Particles spread and converge on the wrong pose,
causing AMCL to publish an incorrect `map→odom` transform. From that point,
every planned path is offset in world space.

**Fix:** In simulation the EKF fuses Gazebo's own ground-truth odometry (`/gz/odom`),
so the `odom` frame already coincides with the world frame — the `map→odom` transform
is the identity. I replaced AMCL's dynamic TF with a static publisher:
```python
Node(package='tf2_ros', executable='static_transform_publisher',
     arguments=['0','0','0','0','0','0','1', 'map', 'odom'])
```
AMCL stays running (the lifecycle manager requires it) but no longer touches the TF:
```yaml
amcl:
  ros__parameters:
    tf_broadcast: false
```
After this, the rover's RViz position matched Gazebo exactly across multi-lap patrols.

**Lesson:** In simulation with ground-truth odometry, AMCL adds noise rather than
accuracy. Whether to trust your odometry or your scan-matching more depends entirely
on which one is more reliable in your environment.

---

### Each waypoint took 7–8 minutes to reach, sometimes never

**Symptom:** The patrol log showed the rover sitting at a single waypoint for over
7 minutes. Occasionally it eventually succeeded; often it timed out entirely.

**First attempt:** I increased the waypoint timeout from 30 s to 300 s.
This only made failures slower, not less frequent.

**Root cause — three interacting problems:**

1. The default BT executed 6 recovery retries. Each retry attempted `BackUp` (reverse
   0.3 m) before spinning. On MarsYard terrain the rover can never back up cleanly —
   the rear chassis hits slopes immediately and backup always failed, consuming the
   entire retry budget on guaranteed-fail operations.

2. Each failed backup triggered a full 360° spin followed by a 5-second wait,
   adding ~150 s of dead time per stuck event.

3. `inflation_radius: 1.0` was inflating so aggressively that A\* could barely thread
   a path through the narrow corridors between rocks and their inflation bubbles.
   The path it did find was essentially a series of tight S-curves, which the controller
   then struggled to follow.

**Fix:** Removed `BackUp` from the BT recovery entirely. Reduced retries from 6 to 3.
Replaced 360° spin with ±90° spin. Reduced inflation to 0.7 m. Added a progress
checker to cancel navigation after 8 s of no movement rather than waiting for the
full retry chain to exhaust:
```yaml
progress_checker:
  required_movement_radius: 0.1
  movement_time_allowance: 8.0
```
Waypoint time dropped from 7–8 min to under 2 min in the majority of cases.

**Lesson:** Recovery behaviours that cannot succeed in your environment are worse
than no recovery at all — they consume time and retry budget on guaranteed failures.
Profile which recoveries actually work before enabling them. Also, inflation radius
and path quality are coupled: over-inflation forces the planner into bad paths that
the controller then repeatedly fails to execute.

---

### Nav2 started cleanly but the rover never moved

**Symptom:** All Nav2 lifecycle nodes were active, a goal was sent, the BT was
ticking, but `/cmd_vel` carried no messages and the rover sat still.

**Root cause:** I had disabled teleop (`with_teleop: false`), assuming twist_mux
was only needed for joystick control. But Nav2's `controller_server` publishes to
`/cmd_vel_nav`, not directly to `/cmd_vel`. `twist_mux` is the bridge between the
two — without it, the robot driver node never receives any velocity command.

**Fix:** `twist_mux` runs unconditionally, with no `with_teleop` condition:
```python
# No condition= — twist_mux must always run regardless of teleop
twist_mux = Node(package='twist_mux', executable='twist_mux', ...)
```
Both Nav2 servers are remapped so their output flows through the mux:
```python
controller = Node(..., remappings=[('/cmd_vel', '/cmd_vel_nav')])
behavior   = Node(..., remappings=[('/cmd_vel', '/cmd_vel_nav')])
```

**Lesson:** `twist_mux` is not a teleop utility — it is the central velocity
routing node. Nav2 and teleop both feed into it. The name is misleading; think
of it as a priority arbiter that all motion sources must pass through.

---

### Nav2 crashed on startup with "bond broken" errors

**Symptom:** The lifecycle manager printed `bond broken` for `map_server` and
shut down the entire Nav2 stack within seconds of launch.

**Root cause:** The lifecycle manager monitors a heartbeat ("bond") with each
managed node. The default timeout is 4 seconds — if a node doesn't respond,
the manager declares it dead and tears down the stack. `map_server` loading the
50×50 m MarsYard occupancy grid takes ~15 seconds before it can respond to any
heartbeat — far past the default.

**Fix:** Disable the bond timeout:
```yaml
lifecycle_manager:
  ros__parameters:
    bond_timeout: 0.0   # wait indefinitely during startup
```

**Lesson:** Nav2's default timeouts assume small test maps. On real-scale
environments, profile how long each node actually takes to initialise and
adjust timeouts accordingly. A 4-second default for a 50-metre world is not realistic.

---

### The slope detection produced silently wrong Z averages

**Symptom:** The slope filter looked correct in unit tests but produced
inconsistent results with real LiDAR data — some flat areas were flagged as
steep, and some actual slopes were missed.

**Root cause:** The initial implementation used standard numpy indexed assignment:
```python
z_sum[xi, yi] += z   # WRONG when xi, yi contain duplicate indices
```
When multiple LiDAR points fall in the same grid cell (which happens constantly
with dense scans), numpy only applies the last update for each index — all prior
contributions for that cell are silently discarded. `z_sum` ended up holding the
value from one arbitrary point per cell, not the true accumulated sum.

**Fix:** `np.add.at` is numpy's unbuffered scatter-add — it processes every
index including duplicates, accumulating all contributions:
```python
np.add.at(z_sum, (xi, yi), z)   # correct: every point is counted
np.add.at(z_cnt, (xi, yi), 1)
z_mean = z_sum / z_cnt
```

**Lesson:** `a[idx] += b` and `np.add.at(a, idx, b)` are not equivalent when
`idx` has repeated values. Any scatter-accumulate pattern — histograms, occupancy
counting, height averaging over a grid — must use `np.add.at` or an explicit loop.
The numpy documentation calls this out, but only if you know to look for it.

---

### The occupancy map had 99% of cells marked as occupied

**Symptom:** After generating `map.png` from the DEM and loading it into Nav2,
RViz showed the entire map as dark grey. A\* refused to plan any path — there
was no free space to route through.

**Root cause:** `build_occupancy_from_dem()` outputs white pixels as obstacles
and black pixels as free space. But `map_server` interprets images with the
opposite convention by default (white = free, black = occupied). Every free cell
in the image was being read as occupied.

**Fix:** One field in `map.yaml` flips the interpretation:
```python
write_map_yaml(map_yaml, 'map.png', resolution, origin, negate=1)
```

**Lesson:** Always add a sanity check after loading a new map: confirm that the
rover's known spawn position appears as free space (white) in RViz. If the entire
map is occupied, the first thing to check is image polarity — not the DEM itself.

---

### The rover moved jerkily and frequently paused mid-path

**Symptom:** Even on flat terrain the rover would drive 1–2 m, slow almost to
zero, pause briefly, then accelerate again — repeated throughout the entire path.

**Root cause:** Two interacting issues:
1. `cost_scaling_dist: 0.8` caused velocity to drop whenever the rover was within
   0.8 m of any costmap obstacle. On MarsYard, slope-derived obstacles surround
   nearly every navigable corridor — the rover was almost permanently inside the
   cost-scaling zone.
2. `desired_linear_vel: 0.8 m/s` was high enough that `cost_scaling_gain: 1.0`
   regularly reduced speed to near zero, producing the pause-then-accelerate pattern.

**Fix:** Widen the cost zone, halve the gain, lower cruise speed:
```yaml
desired_linear_vel: 0.4        # slower baseline = smaller absolute swing
cost_scaling_dist: 1.2         # start slowing earlier, more gradually
cost_scaling_gain: 0.5         # same proximity, half the speed reduction
regulated_linear_scaling_min_speed: 0.1  # never drop below 0.1 m/s
```

**Lesson:** Cost-regulated velocity scaling and inflation radius are tightly
coupled. If inflation is wide (many high-cost cells near the planned path),
velocity scaling becomes effectively permanent rather than situational.
Tune the two parameters together — reducing one often requires adjusting the other.

---

### Controller spawning failed on every launch

**Symptom:** `forward_velocity_controller` and `forward_position_controller`
printed `Could not find joint X in hardware` and exited within seconds of launch.

**Root cause:** The controllers were being spawned in parallel immediately after
Gazebo started. The robot entity did not yet exist in Gazebo when the
`controller_manager` attempted to bind the controllers to its joints.

**Fix:** Chain spawning with `OnProcessExit` event handlers to enforce ordering:
```python
# 1. After gz_spawn_entity exits, wait 1 s then start joint_state_broadcaster
activate_jsb_after_spawn = RegisterEventHandler(
    OnProcessExit(target_action=gz_spawn_entity,
                  on_exit=[TimerAction(period=1.0, actions=[spawner_jsb])]))

# 2. After JSB exits (= active), start remaining controllers in parallel
activate_rest_after_jsb = RegisterEventHandler(
    OnProcessExit(target_action=spawner_jsb,
                  on_exit=[spawner_vel, spawner_pos, spawner_susp, spawner_arm]))
```
`joint_state_broadcaster` must come first because the other controllers depend
on its joint state feedback.

**Lesson:** In ROS 2 launch files there is no implicit ordering between nodes —
everything starts concurrently by default. When a node requires another to be
fully active before it can initialise, that dependency must be encoded explicitly
with `OnProcessExit` or `OnProcessStart` handlers.

---

### Understanding what actually changes between `mapping` and `localization` mode

This took a while to understand concretely. Switching `mode:=mapping` is not just
"turn SLAM on" — it changes which nodes run and which node is responsible for the
`map→odom` TF:

| | `localization` | `mapping` |
|--|---------------|-----------|
| Map source | `map_server` loads `map.png` from disk | `slam_toolbox` builds the map live |
| Pose estimate | AMCL particle filter against the static map | slam_toolbox provides pose directly |
| `map→odom` TF | Static identity publisher (simulation bypass) | slam_toolbox publishes it |
| Lifecycle node list | includes `map_server` + `amcl` | excludes them |
| When to use | Known environment — repeatable patrol | Unknown terrain — first exploration |

Two separate lifecycle managers exist (one per mode) because they manage different
`node_names` lists. Only one runs at a time, selected by the `mode` launch argument.

---

*Morpheus — Mars rover autonomous navigation (ROS 2 Humble / Ignition Gazebo Fortress)*
