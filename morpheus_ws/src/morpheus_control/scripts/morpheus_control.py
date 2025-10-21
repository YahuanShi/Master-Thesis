#!/usr/bin/python3
import math
import threading

from geometry_msgs.msg import Twist
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration  # [3A] for timeout
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# ----------------- 全局共享（为保持你原始结构的最小改动） -----------------
vel_msg_joy = Twist()   # 手柄速度
vel_msg_nav = Twist()   # Nav2 速度
mode_selection = 4      # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none (drive)
auto_mode = False       # [3A] False=手柄优先, True=Nav2优先
last_nav_stamp_ns = 0   # [3A] 记录最近一次收到 /cmd_vel_nav 的时刻 (nanoseconds)

class Robot_control(Node):

    def __init__(self):

        super().__init__('robot_control')

        timer_period = 0.01

        # ----------------
        self.wheel_base = 1.072
        self.wheel_radius = 0.125
        self.wheel_steering_y_offset = 0.0
        self.wheel_seperation = 0.615
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        # -----------------------------------------------------------
        # Initialization
        self.pos = np.array([0, 0, 0, 0], float)
        self.vel = np.array([0, 0, 0, 0], float)  # left_front, right_front, left_rear, right_rear
        self.ra_pos = np.array([0, 0, 0, 0, 0, 0, 0], float)

        # Defining Controller Command Publishers
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_ra_pos = self.create_publisher(Float64MultiArray, '/robotic_arm_controller/commands', 10)
        # --------------------------------------
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection

        self.deadzone = 0.1

        # Apply deadzone for smoother joystick control
        speed = math.sqrt(vel_msg.linear.x**2 + vel_msg.linear.y**2)
        scale = max(0, (speed - self.deadzone) / (1 - self.deadzone)) / speed if speed > self.deadzone else 0

        if mode_selection == 4:
            if abs(vel_msg.linear.x) <= self.deadzone:
                vel_msg.linear.x *= scale
            if abs(vel_msg.linear.y) <= self.deadzone:
                vel_msg.linear.y *= scale
            speed = math.sqrt(vel_msg.angular.z**2)
            scale = max(0, (speed - self.deadzone) / (1 - self.deadzone)) / speed if speed > self.deadzone else 0
            if abs(vel_msg.angular.z) <= self.deadzone:
                vel_msg.angular.z *= scale

            if vel_msg.angular.z != 0:
                if vel_msg.linear.x != 0:
                    # Ackermann
                    factor = 10

                    if abs(vel_msg.angular.z) < 1e-5:  # Avoid division by zero
                        self.pos[:] = 0.0 
                    else:
                        r = abs(vel_msg.linear.x) / vel_msg.angular.z * 2*math.pi
                        r_bl = r + self.steering_track / 2
                        r_br = r - self.steering_track / 2

                        # Calculate the steering angles for the front left and front right wheels
                        a_fl = math.atan(self.wheel_base / r_bl)
                        a_fr = math.atan(self.wheel_base / r_br)

                        # Adjust steering angles when point of rotation is between the wheels (there we want >90° steering angle)
                        if r_bl > 0 and r < 0:
                            a_fl -= math.pi
                        if r_br < 0 and r > 0:
                            a_fr += math.pi

                        self.pos[0] = a_fl * 1.57
                        self.pos[1] = a_fr * 1.57

                    vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
                    sign = np.sign(vel_msg.linear.x)

                    self.vel[0] = sign*math.hypot(
                        vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2,
                        vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
                    self.vel[1] = sign*math.hypot(
                        vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2,
                        vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
                    self.vel[2] = sign*math.hypot(
                        vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2,
                        vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
                    self.vel[3] = sign*math.hypot(
                        vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2,
                        vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
                    
                    self.vel[:] *= factor
                    
                else:
                    # Pivot Turn
                    factor = 10

                    self.pos[0] = -math.atan(self.wheel_base/self.steering_track)
                    self.pos[1] = math.atan(self.wheel_base/self.steering_track)
                    self.pos[2] = math.atan(self.wheel_base/self.steering_track)
                    self.pos[3] = -math.atan(self.wheel_base/self.steering_track)

                    self.vel[0] = -vel_msg.angular.z
                    self.vel[1] = vel_msg.angular.z 
                    self.vel[2] = self.vel[0]
                    self.vel[3] = self.vel[1]

                    self.vel[:] *= factor

            else:
                # Crab Walk
                factor = 10

                if vel_msg.linear.x <= 0.0 and vel_msg.linear.y == 0.0:
                    self.pos[:] = 0.0
                else:
                    angle = math.atan2(vel_msg.linear.y, vel_msg.linear.x)
                    if abs(angle) >= math.pi/2:
                        angle = (-1)*np.sign(angle)*(math.pi - abs(angle))
                    self.pos[:] = angle

                magnitude = math.sqrt(vel_msg.linear.x**2 + vel_msg.linear.y**2)
                sign = 1 if vel_msg.linear.x > 0 else -1
                self.vel[:] = magnitude * sign * factor

        # Robotic Arm Modes
        if(mode_selection == 5):
            if int(vel_msg.linear.y) != 0:
                command = self.ra_pos[0] - vel_msg.linear.y * 0.005 # velocity * timer_period
                if abs(command) < 1.5:
                    self.ra_pos[0] = command
                else: 
                    if np.sign(vel_msg.linear.y)*np.sign(self.ra_pos[0]) < 0:
                        self.ra_pos[0] = np.sign(self.ra_pos[0])*1.5
                    else:
                        self.ra_pos[0] = command

        elif(mode_selection == 6):
            if int(vel_msg.linear.x) != 0:
                command = self.ra_pos[1] - vel_msg.linear.x * 0.005 # velocity * timer_period
                if abs(command) < 1.5:
                    self.ra_pos[1] = command
                else: 
                    if np.sign(vel_msg.linear.x)*np.sign(self.ra_pos[1]) < 0:
                        self.ra_pos[1] = np.sign(self.ra_pos[1])*1.5
                    else:
                        self.ra_pos[1] = command

        elif(mode_selection == 7):
            if int(vel_msg.linear.x) != 0:
                command = self.ra_pos[2] - vel_msg.linear.x * 0.005 # velocity * timer_period
                if abs(command) < 1.5:
                    self.ra_pos[2] = command
                else: 
                    if np.sign(vel_msg.linear.x)*np.sign(self.ra_pos[2]) < 0:
                        self.ra_pos[2] = np.sign(self.ra_pos[2])*1.5
                    else:
                        self.ra_pos[2] = command

        elif(mode_selection == 8):
            if int(vel_msg.angular.y) != 0:
                command = self.ra_pos[3] - vel_msg.angular.y * 0.005 # velocity * timer_period
                if abs(command) < 1.5:
                    self.ra_pos[3] = command
                else: 
                    if np.sign(vel_msg.angular.y)*np.sign(self.ra_pos[3]) < 0:
                        self.ra_pos[3] = np.sign(self.ra_pos[3])*1.5
                    else:
                        self.ra_pos[3] = command

        elif(mode_selection == 9):
            if int(vel_msg.angular.z) != 0:
                command = self.ra_pos[4] - vel_msg.angular.z * 0.005 # velocity * timer_period
                if abs(command) < 1.5:
                    self.ra_pos[4] = command
                else: 
                    if np.sign(vel_msg.angular.z)*np.sign(self.ra_pos[4]) < 0:
                        self.ra_pos[4] = np.sign(self.ra_pos[4])*1.5
                    else:
                        self.ra_pos[4] = command

        elif(mode_selection == 10):
            if int(vel_msg.angular.z) != 0:
                command = self.ra_pos[5] - vel_msg.angular.z * 0.005 # velocity * timer_period
                if abs(command) < 1.5:
                    self.ra_pos[5] = command
                else: 
                    if np.sign(vel_msg.angular.z)*np.sign(self.ra_pos[5]) < 0:
                        self.ra_pos[5] = np.sign(self.ra_pos[5])*1.5
                    else:
                        self.ra_pos[5] = command


        if(mode_selection == 11):
            if round(vel_msg.angular.z,3) != 0: # round statt int
                command = self.ra_pos[6] - vel_msg.angular.z * 0.005 # velocity * timer_period
                if command >= 0.0:
                    if abs(command) < 0.29:
                        self.ra_pos[6] = command
                    else: 
                        if np.sign(vel_msg.angular.z)*np.sign(self.ra_pos[6]) < 0:
                            self.ra_pos[6] = np.sign(self.ra_pos[6])*0.29
                        else:
                            self.ra_pos[6] = command
                else:
                    self.ra_pos[6] = 0

        pos_array = Float64MultiArray(data=self.pos)
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.pos[:] = 0.0
        self.vel[:] = 0.0
        ra_pos_array = Float64MultiArray(data=self.ra_pos)
        self.pub_ra_pos.publish(ra_pos_array)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy, 'joy', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection
        if data.axes:
            if(data.buttons[1] == 1):    # robotic arm base
                # B button of Xbox controller
                mode_selection = 5
            elif(data.buttons[3] == 1):    # shoulder
                # Y button of Xbox controller
                mode_selection = 6
            elif(data.buttons[2] == 1):    # elbow
                # X button of Xbox controller
                mode_selection = 7
            elif(data.axes[6] == -1):      # wrist 1
                # right button of Xbox controller
                mode_selection = 8
            elif(data.axes[7] == 1):       # wrist 2
                # arrow up of Xbox controller
                mode_selection = 9
            elif(data.axes[6] == 1):       # wrist 3
                # arrow left of Xbox controller
                mode_selection = 10
            elif(data.axes[7] == -1):      # gripper
                # arrow down of Xbox controller
                mode_selection = 11
            else: # drive rover
                mode_selection = 4

            vel_msg.linear.x = data.axes[1]
            vel_msg.linear.y = data.axes[0]

            vel_msg.angular.z = data.axes[3] 
            vel_msg.angular.y = data.axes[4]
        
if __name__ == '__main__':
    rclpy.init(args=None)

    robot_control = Robot_control()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_control)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = robot_control.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
