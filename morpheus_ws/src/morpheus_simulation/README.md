# morpheus_simulation

Gazebo worlds, robot spawning, and ROS-Gazebo bridges.

## Worlds

- `marsyard2022.sdf` — Mars Yard terrain with 6 ArUco boxes and 1 flat ArUco marker
- `empty_world.sdf` — Flat ground plane with basic obstacles for testing

## Launch

```bash
ros2 launch morpheus_simulation morpheus_spawn.launch.py
```

### Key Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `marsyard2022` | World SDF name (without .sdf) |
| `spawn_x/y/z` | `0.0 / 4.0 / 1.5` | Rover spawn position |
| `mode` | `localization` | Nav2 mode: `localization` or `mapping` |
| `with_rviz` | `false` | Launch RViz2 |
| `with_teleop` | `true` | Launch teleop + twist_mux |
| `gui` | `true` | Start Gazebo GUI |

### Scene Switching

```bash
# Mars Yard (default)
ros2 launch morpheus_simulation morpheus_spawn.launch.py

# Empty world for quick tests
ros2 launch morpheus_simulation morpheus_spawn.launch.py world:=empty_world spawn_z:=0.5
```

## Bridges

The launch file creates bridges for: clock, LiDAR scan+points, camera RGB+info+depth+points, IMU, and Gazebo odometry. The odom bridge dynamically constructs its Gazebo topic from the `world` argument and remaps to `/gz/odom`.
