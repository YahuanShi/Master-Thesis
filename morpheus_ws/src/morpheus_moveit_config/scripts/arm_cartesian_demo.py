#!/usr/bin/env python3
"""Demo node: plan and execute cartesian pose goals for the Morpheus arm.

Uses MoveIt2's MoveGroup action interface to send target poses for the
end effector and execute planned trajectories via the arm trajectory
controller.

Usage (after launching moveit.launch.py):
  ros2 run morpheus_moveit_config arm_cartesian_demo.py
  ros2 run morpheus_moveit_config arm_cartesian_demo.py --ros-args -p target_x:=0.3
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class ArmCartesianDemo(Node):
    def __init__(self):
        super().__init__('arm_cartesian_demo')

        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('target_x', 0.3)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 0.4)
        self.declare_parameter('planning_time', 5.0)

        self._group = self.get_parameter('planning_group').value
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected. Sending cartesian goal.')
        self._send_goal()

    def _send_goal(self):
        target = PoseStamped()
        target.header.frame_id = 'chassis_link'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = self.get_parameter('target_x').value
        target.pose.position.y = self.get_parameter('target_y').value
        target.pose.position.z = self.get_parameter('target_z').value
        target.pose.orientation.w = 1.0

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self._group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = self.get_parameter('planning_time').value
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header = target.header
        pos_constraint.link_name = 'wrist_3_link'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.01]
        bv.primitives.append(prim)
        bv.primitive_poses.append(target.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header = target.header
        ori_constraint.link_name = 'wrist_3_link'
        ori_constraint.orientation = target.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 0.5
        constraints.orientation_constraints.append(ori_constraint)

        goal.request.goal_constraints.append(constraints)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self._future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        self._future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        state = feedback_msg.feedback.state
        self.get_logger().info(f'Planning state: {state}')

    def _result_cb(self, future):
        result = future.result().result
        error = result.error_code.val
        if error == 1:
            self.get_logger().info('Motion executed successfully!')
        else:
            self.get_logger().error(f'Motion failed with error code: {error}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmCartesianDemo()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
