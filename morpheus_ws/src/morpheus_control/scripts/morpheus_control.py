#!/usr/bin/python3
import math
import threading

from geometry_msgs.msg import Twist
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# Lock-protected shared state between Joy_subscriber and Robot_control.
# Joy_subscriber writes; Robot_control reads inside its timer callback.
_lock = threading.Lock()
_joy_axes = Twist()
_mode = 4  # 4: drive (chassis); 5..11: robotic arm joints


class Robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        self.wheel_base = 1.072
        self.wheel_radius = 0.125
        self.wheel_steering_y_offset = 0.0
        self.wheel_seperation = 0.615
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.drive_gain = float(self.declare_parameter('drive_gain', 10.0).value)

        self.pos = np.array([0.0, 0.0, 0.0, 0.0], float)
        self.vel = np.array([0.0, 0.0, 0.0, 0.0], float)
        self.ra_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], float)

        self.base_cmd = Twist()

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_ra_pos = self.create_publisher(Float64MultiArray, '/robotic_arm_controller/commands', 10)

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.deadzone = 0.05

    def cmd_vel_cb(self, msg: Twist):
        self.base_cmd = msg

    def timer_callback(self):
        with _lock:
            mode_selection = _mode
            joy_lx = _joy_axes.linear.x
            joy_ly = _joy_axes.linear.y
            joy_az = _joy_axes.angular.z
            joy_ay = _joy_axes.angular.y

        if mode_selection == 4:
            vx = self.base_cmd.linear.x
            vy = self.base_cmd.linear.y
            wz = self.base_cmd.angular.z

            if abs(vx) < self.deadzone: vx = 0.0
            if abs(vy) < self.deadzone: vy = 0.0
            if abs(wz) < self.deadzone: wz = 0.0

            if wz != 0.0:
                if vx != 0.0:
                    # Ackermann mixed steering
                    factor = self.drive_gain

                    if abs(wz) < 1e-5:
                        self.pos[:] = 0.0
                    else:
                        r = abs(vx) / wz * 2*math.pi
                        r_bl = r + self.steering_track / 2.0
                        r_br = r - self.steering_track / 2.0

                        a_fl = math.atan(self.wheel_base / r_bl)
                        a_fr = math.atan(self.wheel_base / r_br)

                        if r_bl > 0 and r < 0:
                            a_fl -= math.pi
                        if r_br < 0 and r > 0:
                            a_fr += math.pi

                        self.pos[0] = a_fl * 1.57
                        self.pos[1] = a_fr * 1.57

                    vel_steerring_offset = wz * self.wheel_steering_y_offset
                    sign = np.sign(vx) if vx != 0.0 else 1.0

                    self.vel[0] = sign*math.hypot(vx - wz*self.steering_track/2.0,
                                                   wz*self.wheel_base/2.0) - vel_steerring_offset
                    self.vel[1] = sign*math.hypot(vx + wz*self.steering_track/2.0,
                                                   wz*self.wheel_base/2.0) + vel_steerring_offset
                    self.vel[2] = self.vel[0]
                    self.vel[3] = self.vel[1]
                    self.vel[:] *= factor

                else:
                    # Pivot turn
                    factor = self.drive_gain
                    ang = math.atan(self.wheel_base/self.steering_track)
                    self.pos[0] = -ang
                    self.pos[1] =  ang
                    self.pos[2] =  ang
                    self.pos[3] = -ang
                    self.vel[0] = -wz
                    self.vel[1] =  wz
                    self.vel[2] = -wz
                    self.vel[3] =  wz
                    self.vel[:] *= factor
            else:
                # Crab walk
                factor = self.drive_gain
                if vx <= 0.0 and vy == 0.0:
                    self.pos[:] = 0.0
                else:
                    angle = math.atan2(vy, vx) if (vx != 0.0 or vy != 0.0) else 0.0
                    if abs(angle) >= math.pi/2:
                        angle = (-1)*np.sign(angle)*(math.pi - abs(angle))
                    self.pos[:] = angle

                magnitude = math.hypot(vx, vy)
                sign = 1.0 if vx > 0.0 else -1.0 if vx < 0.0 else 1.0
                self.vel[:] = magnitude * sign * factor

        # Robotic arm joint control
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

        elif mode_selection == 10:
            if int(joy_az) != 0:
                command = self.ra_pos[5] - joy_az * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[5] = command
                else:
                    if np.sign(joy_az)*np.sign(self.ra_pos[5]) < 0:
                        self.ra_pos[5] = np.sign(self.ra_pos[5]) * 1.5
                    else:
                        self.ra_pos[5] = command

        if mode_selection == 11:
            if round(joy_az, 3) != 0:
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

        if data.buttons[1] == 1:       # B -> robotic arm base
            mode = 5
        elif data.buttons[3] == 1:     # Y -> shoulder
            mode = 6
        elif data.buttons[2] == 1:     # X -> elbow
            mode = 7
        elif data.axes[6] == -1:       # d-pad right -> wrist_1
            mode = 8
        elif data.axes[7] == 1:        # d-pad up -> wrist_2
            mode = 9
        elif data.axes[6] == 1:        # d-pad left -> wrist_3
            mode = 10
        elif data.axes[7] == -1:       # d-pad down -> gripper
            mode = 11
        else:
            mode = 4

        axes = Twist()
        axes.linear.x  = data.axes[1]
        axes.linear.y  = data.axes[0]
        axes.angular.z = data.axes[3]
        axes.angular.y = data.axes[4]

        with _lock:
            _mode = mode
            _joy_axes = axes


if __name__ == '__main__':
    rclpy.init(args=None)

    robot_control = Robot_control()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_control)
    executor.add_node(joy_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
