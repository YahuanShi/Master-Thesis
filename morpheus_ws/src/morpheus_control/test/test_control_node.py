#!/usr/bin/env python3
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='morpheus_control',
        executable='morpheus_control.py',
        name='robot_control_test',
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'control_node': node}


class TestControlNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_control_node')
        cls.pub = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.received_pos = None
        cls.received_vel = None

        def pos_cb(msg):
            cls.received_pos = list(msg.data)

        def vel_cb(msg):
            cls.received_vel = list(msg.data)

        cls.sub_pos = cls.node.create_subscription(
            Float64MultiArray,
            '/forward_position_controller/commands',
            pos_cb, 10)
        cls.sub_vel = cls.node.create_subscription(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            vel_cb, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, check, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if check():
                return True
        return False

    def test_node_publishes_on_startup(self):
        ok = self._spin_until(
            lambda: self.received_pos is not None and self.received_vel is not None)
        self.assertTrue(ok, 'Control node did not publish within timeout')

    def test_forward_cmd_vel(self):
        ok = self._spin_until(
            lambda: self.pub.get_subscription_count() > 0, timeout=10.0)
        self.assertTrue(ok, 'cmd_vel publisher has no matched subscriptions')

        msg = Twist()
        msg.linear.x = 1.0
        type(self).received_vel = None

        def check_and_publish():
            self.pub.publish(msg)
            return (self.received_vel is not None
                    and any(v != 0.0 for v in self.received_vel))

        ok = self._spin_until(check_and_publish, timeout=15.0)
        self.assertTrue(ok, 'No non-zero velocity after publishing cmd_vel')
        self.assertEqual(len(self.received_pos), 4)
        self.assertEqual(len(self.received_vel), 4)
        for v in self.received_vel:
            self.assertGreater(v, 0.0, 'Expected positive velocity for forward cmd')
