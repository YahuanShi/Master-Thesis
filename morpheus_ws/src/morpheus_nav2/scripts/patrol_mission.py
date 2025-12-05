#!/usr/bin/env python3
"""Patrol mission — sequentially navigate to waypoints, checking for ArUco markers at each stop."""

import time
import yaml

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory

try:
    from ros2_aruco_interfaces.msg import ArucoMarkers
    _HAS_ARUCO = True
except ImportError:
    _HAS_ARUCO = False


class PatrolMission:
    def __init__(self):
        self.nav = BasicNavigator('patrol_mission')

        self.nav.declare_parameter('waypoints_file', '')
        self.nav.declare_parameter('loop', True)
        self.nav.declare_parameter('dwell_time', 3.0)
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

        self.recent_markers: list[int] = []
        if _HAS_ARUCO:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.nav.create_subscription(
                ArucoMarkers, '/aruco_detector_2i/aruco_markers',
                self._aruco_cb, qos)

    def _load_waypoints(self, path: str) -> list[dict]:
        self.nav.get_logger().info(f'Loading waypoints from {path}')
        with open(path) as f:
            data = yaml.safe_load(f)
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
        q = quaternion_from_euler(0.0, 0.0, float(wp.get('yaw', 0.0)))
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        return goal

    def run(self):
        log = self.nav.get_logger()

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

                while not self.nav.isTaskComplete():
                    rclpy.spin_once(self.nav, timeout_sec=0.1)

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
