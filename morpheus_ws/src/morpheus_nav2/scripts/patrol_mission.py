#!/usr/bin/env python3
"""Patrol mission — sequentially navigate to waypoints, checking for ArUco markers at each stop."""

import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf_transformations import quaternion_from_euler

try:
    from ros2_aruco_interfaces.msg import ArucoMarkers
    _HAS_ARUCO = True
except ImportError:
    _HAS_ARUCO = False


class PatrolMission:
    def __init__(self):
        # BasicNavigator wraps the Nav2 action clients (NavigateToPose, etc.)
        # and provides a synchronous-style API on top of ROS 2 async actions
        self.nav = BasicNavigator('patrol_mission')

        self.nav.declare_parameter('waypoints_file', '')
        self.nav.declare_parameter('loop', True)
        self.nav.declare_parameter('dwell_time', 3.0)
        self.nav.declare_parameter('waypoint_timeout', 150.0)
        self.nav.declare_parameter('initial_x', 0.0)
        self.nav.declare_parameter('initial_y', 4.0)
        self.nav.declare_parameter('initial_yaw', 0.0)

        wf = self.nav.get_parameter('waypoints_file').value
        if not wf:
            pkg = get_package_share_directory('morpheus_nav2')
            wf = pkg + '/config/patrol_waypoints.yaml'
        self.waypoints = self._load_waypoints(wf)
        self.loop = self.nav.get_parameter('loop').value
        self.dwell_time = self.nav.get_parameter('dwell_time').value
        self.waypoint_timeout = self.nav.get_parameter('waypoint_timeout').value

        self.recent_markers: list[int] = []
        if _HAS_ARUCO:
            # BEST_EFFORT QoS to match the ArUco publisher; RELIABLE would cause
            # the subscription to never receive messages if publishers differ
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.nav.create_subscription(
                ArucoMarkers, '/aruco_detector_2i/aruco_markers',
                self._aruco_cb, qos)

    def _load_waypoints(self, path: str) -> list[dict]:
        self.nav.get_logger().info(f'Loading waypoints from {path}')
        with open(path) as f:
            data = yaml.safe_load(f)
        # Support two YAML layouts: flat 'waypoints' list or ROS param-style nesting
        wps = data.get('waypoints', [])
        if not wps:
            raw = data.get('patrol_mission', {}).get('ros__parameters', {})
            wps = raw.get('waypoints', [])
        if not wps:
            self.nav.get_logger().error('No waypoints found in config')
        return wps

    def _aruco_cb(self, msg: 'ArucoMarkers'):
        self.recent_markers = list(msg.marker_ids)

    def _make_goal(self, wp: dict) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = float(wp['x'])
        goal.pose.position.y = float(wp['y'])
        # Convert yaw to quaternion — Nav2 expects full orientation, not just yaw
        q = quaternion_from_euler(0.0, 0.0, float(wp.get('yaw', 0.0)))
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        return goal

    def run(self):
        log = self.nav.get_logger()

        # Publish an initial pose estimate to AMCL so the particle filter starts
        # near the true spawn position rather than spreading across the whole map
        ix = self.nav.get_parameter('initial_x').value
        iy = self.nav.get_parameter('initial_y').value
        iyaw = self.nav.get_parameter('initial_yaw').value
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.nav.get_clock().now().to_msg()
        init_pose.pose.position.x = float(ix)
        init_pose.pose.position.y = float(iy)
        q = quaternion_from_euler(0.0, 0.0, float(iyaw))
        init_pose.pose.orientation.z = q[2]
        init_pose.pose.orientation.w = q[3]
        self.nav.setInitialPose(init_pose)
        log.info(f'Initial pose: ({ix}, {iy}, yaw={iyaw})')

        # Block until all Nav2 lifecycle nodes are in the 'active' state.
        # localizer='amcl' means it specifically waits for AMCL to activate,
        # not just the planner/controller. This matters because AMCL loads the
        # map and sets up the particle filter before it transitions to active.
        log.info('Waiting for Nav2...')
        self.nav.waitUntilNav2Active(localizer='amcl')

        lap = 0
        while rclpy.ok():
            lap += 1
            log.info(f'=== Patrol lap {lap} ===')

            for i, wp in enumerate(self.waypoints):
                name = wp.get('name', f'wp_{i}')
                log.info(f'[{i+1}/{len(self.waypoints)}] Navigating to {name} '
                         f'({wp["x"]:.1f}, {wp["y"]:.1f})')

                goal = self._make_goal(wp)
                self.nav.goToPose(goal)

                # Per-waypoint timeout using monotonic time (not ROS sim time)
                # so it always counts real wall-clock seconds, unaffected by
                # Gazebo time scale or pauses
                deadline = time.monotonic() + self.waypoint_timeout
                while not self.nav.isTaskComplete():
                    rclpy.spin_once(self.nav, timeout_sec=0.1)
                    if time.monotonic() > deadline:
                        log.warn(f'Timeout ({self.waypoint_timeout:.0f}s) reaching {name}, canceling')
                        self.nav.cancelTask()
                        break

                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    log.info(f'Reached {name} — dwelling {self.dwell_time:.0f}s')
                    self._dwell_and_detect(name)
                elif result == TaskResult.CANCELED:
                    log.warn('Mission canceled')
                    return
                else:
                    log.warn(f'Failed to reach {name}, skipping')

            if not self.loop:
                log.info('Patrol complete (single pass)')
                return

    def _dwell_and_detect(self, wp_name: str):
        """Pause at a waypoint and collect any ArUco marker observations.

        Uses spin_once in a loop rather than time.sleep so that the ROS callback
        queue keeps draining — without this, incoming ArUco messages would queue
        up and never be processed during the dwell period.
        """
        log = self.nav.get_logger()
        self.recent_markers.clear()

        t0 = time.monotonic()
        while time.monotonic() - t0 < self.dwell_time:
            rclpy.spin_once(self.nav, timeout_sec=0.2)

        if self.recent_markers:
            log.info(f'ArUco markers detected at {wp_name}: {self.recent_markers}')
        else:
            log.info(f'No ArUco markers detected at {wp_name}')


def main():
    rclpy.init()
    mission = PatrolMission()
    try:
        mission.run()
    except KeyboardInterrupt:
        pass
    finally:
        mission.nav.lifecycleShutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
