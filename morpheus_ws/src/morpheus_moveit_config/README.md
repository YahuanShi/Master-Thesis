# morpheus_moveit_config

MoveIt2 configuration for the Morpheus rover's 6-DOF robotic arm.

## Setup

- **arm** group: chain from `chassis_link` to `wrist_3_link` (6 joints)
- **gripper** group: `gripper_left_joint`
- **End effector**: gripper, attached to `wrist_3_link`
- **Kinematics solver**: KDL
- **Planner**: OMPL (RRTConnect + RRTstar)
- **Controllers**: JointTrajectoryController for arm and gripper

## Named Poses

| Pose | Description |
|---|---|
| `home` | All arm joints at zero |
| `stow` | Arm folded for transport |
| `open` | Gripper open |
| `closed` | Gripper closed |

## Demo

```bash
ros2 launch morpheus_moveit_config moveit.launch.py
ros2 run morpheus_moveit_config arm_cartesian_demo.py
```

The demo sends a Cartesian pose goal to the MoveGroup action server.
