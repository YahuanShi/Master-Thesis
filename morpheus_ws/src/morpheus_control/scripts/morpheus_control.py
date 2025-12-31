#!/usr/bin/python3
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from kinematics import ChassisParams, compute_drive
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# Shared state written by Joy_subscriber and read by Robot_control.
# Both nodes run in a MultiThreadedExecutor, so a lock prevents torn reads.
_lock = threading.Lock()
_joy_axes = Twist()
_mode = 4  # 4: chassis drive; 5–11: robotic arm joints (one joint per mode)


class Robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.drive_gain = float(self.declare_parameter('drive_gain', 10.0).value)
        self.chassis = ChassisParams(drive_gain=self.drive_gain)

        # Steering positions (rad, controller units) and wheel velocities for 4 wheels
        self.pos = np.array([0.0, 0.0, 0.0, 0.0], float)
        self.vel = np.array([0.0, 0.0, 0.0, 0.0], float)
        # Robotic arm: 6 joints + 1 gripper = 7 DOF
        self.ra_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], float)

        self.base_cmd = Twist()

        # ros2_control forward controllers: position for steering joints,
        # velocity for drive wheels, position for arm joints
        self.pub_pos = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_ra_pos = self.create_publisher(
            Float64MultiArray, '/robotic_arm_controller/commands', 10)

        # Nav2 / teleop commands arrive on /cmd_vel (after twist_mux priority merge)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 100 Hz control loop: reads latest cmd + joy state and publishes to controllers
        self.timer = self.create_timer(0.01, self.timer_callback)

    def cmd_vel_cb(self, msg: Twist):
        self.base_cmd = msg

    def timer_callback(self):
        # Snapshot shared joy state under the lock to avoid a torn read between
        # mode and axes (Joy_subscriber writes both atomically)
        with _lock:
            mode_selection = _mode
            joy_lx = _joy_axes.linear.x
            joy_ly = _joy_axes.linear.y
            joy_az = _joy_axes.angular.z
            joy_ay = _joy_axes.angular.y

        if mode_selection == 4:
            # Chassis drive: convert Twist → per-wheel steer + velocity via kinematics
            self.pos, self.vel = compute_drive(
                self.base_cmd.linear.x,
                self.base_cmd.linear.y,
                self.base_cmd.angular.z,
                self.chassis,
            )

        # Arm joint control (modes 5–11): each mode maps one joystick axis to one joint.
        # Integrator pattern: joystick deflection increments the target position each tick.
        # Hard limit ±1.5 rad (all arm joints); gripper limited to [0, 0.29] rad.
        # The sign check on np.sign(joy)*np.sign(pos) prevents "sticking" at the limit:
        # if joystick pushes further into the limit, we clamp; if it reverses, we allow.
        if mode_selection == 5:
            if int(joy_ly) != 0:
                command = self.ra_pos[0] - joy_ly * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[0] = command
                else:
                    if np.sign(joy_ly)*np.sign(self.ra_pos[0]) < 0:
                        self.ra_pos[0] = np.sign(self.ra_pos[0]) * 1.5
                    else:
                        self.ra_pos[0] = command

        elif mode_selection == 6:
            if int(joy_lx) != 0:
                command = self.ra_pos[1] - joy_lx * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[1] = command
                else:
                    if np.sign(joy_lx)*np.sign(self.ra_pos[1]) < 0:
                        self.ra_pos[1] = np.sign(self.ra_pos[1]) * 1.5
                    else:
                        self.ra_pos[1] = command

        elif mode_selection == 7:
            if int(joy_lx) != 0:
                command = self.ra_pos[2] - joy_lx * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[2] = command
                else:
                    if np.sign(joy_lx)*np.sign(self.ra_pos[2]) < 0:
                        self.ra_pos[2] = np.sign(self.ra_pos[2]) * 1.5
                    else:
                        self.ra_pos[2] = command

        elif mode_selection == 8:
            if int(joy_ay) != 0:
                command = self.ra_pos[3] - joy_ay * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[3] = command
                else:
                    if np.sign(joy_ay)*np.sign(self.ra_pos[3]) < 0:
                        self.ra_pos[3] = np.sign(self.ra_pos[3]) * 1.5
                    else:
                        self.ra_pos[3] = command

        elif mode_selection == 9:
            if int(joy_az) != 0:
                command = self.ra_pos[4] - joy_az * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[4] = command
                else:
                    if np.sign(joy_az)*np.sign(self.ra_pos[4]) < 0:
                        self.ra_pos[4] = np.sign(self.ra_pos[4]) * 1.5
                    else:
                        self.ra_pos[4] = command

        elif mode_selection == 10 and int(joy_az) != 0:
            command = self.ra_pos[5] - joy_az * 0.005
            if abs(command) < 1.5:
                self.ra_pos[5] = command
            else:
                if np.sign(joy_az)*np.sign(self.ra_pos[5]) < 0:
                    self.ra_pos[5] = np.sign(self.ra_pos[5]) * 1.5
                else:
                    self.ra_pos[5] = command

        if mode_selection == 11 and round(joy_az, 3) != 0:
            # Gripper: one-directional limit (only positive opening, closes to 0)
            command = self.ra_pos[6] - joy_az * 0.005
            if command >= 0.0:
                if abs(command) < 0.29:
                    self.ra_pos[6] = command
                else:
                    if np.sign(joy_az)*np.sign(self.ra_pos[6]) < 0:
                        self.ra_pos[6] = np.sign(self.ra_pos[6]) * 0.29
                    else:
                        self.ra_pos[6] = command
            else:
                self.ra_pos[6] = 0.0

        self.pub_pos.publish(Float64MultiArray(data=self.pos))
        self.pub_vel.publish(Float64MultiArray(data=self.vel))
        self.pub_ra_pos.publish(Float64MultiArray(data=self.ra_pos))

        # Reset wheel commands each tick: if no new /cmd_vel arrives, wheels stop.
        # Arm positions are NOT reset — they hold their last commanded position.
        self.pos[:] = 0.0
        self.vel[:] = 0.0


class Joy_subscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)

    def listener_callback(self, data: Joy):
        global _joy_axes, _mode
        if not data.axes:
            return

        # Button/D-pad → mode mapping (Xbox controller layout)
        if data.buttons[1] == 1:       # B → arm base (joint 0)
            mode = 5
        elif data.buttons[3] == 1:     # Y → shoulder (joint 1)
            mode = 6
        elif data.buttons[2] == 1:     # X → elbow (joint 2)
            mode = 7
        elif data.axes[6] == -1:       # D-pad right → wrist_1 (joint 3)
            mode = 8
        elif data.axes[7] == 1:        # D-pad up → wrist_2 (joint 4)
            mode = 9
        elif data.axes[6] == 1:        # D-pad left → wrist_3 (joint 5)
            mode = 10
        elif data.axes[7] == -1:       # D-pad down → gripper (joint 6)
            mode = 11
        else:
            mode = 4                   # default: chassis drive

        # Pack joystick axes into a Twist for convenient cross-thread passing
        axes = Twist()
        axes.linear.x = data.axes[1]   # left stick Y
        axes.linear.y = data.axes[0]   # left stick X
        axes.angular.z = data.axes[3]  # right stick X
        axes.angular.y = data.axes[4]  # right stick Y

        with _lock:
            _mode = mode
            _joy_axes = axes


if __name__ == '__main__':
    rclpy.init(args=None)

    robot_control = Robot_control()
    joy_subscriber = Joy_subscriber()

    # MultiThreadedExecutor lets both nodes spin concurrently in separate threads.
    # The _lock above ensures the shared joy state is accessed safely.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_control)
    executor.add_node(joy_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        robot_control.destroy_node()
        joy_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
