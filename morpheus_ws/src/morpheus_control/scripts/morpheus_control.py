#!/usr/bin/python3
import math
import threading

from geometry_msgs.msg import Twist
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

# -------------------
# 全局，仅供 Joy_subscriber 与 Robot_control 共享机械臂输入
# （注意：底盘不再使用 vel_msg；底盘使用 /cmd_vel 回调中的 self.base_cmd）
# -------------------
vel_msg = Twist()        # 仅用作机械臂控制的“输入缓存”（来自手柄轴）
mode_selection = 4       # 4: 驾驶(底盘)；5..11: 机械臂各关节

class Robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')

        # 轮系参数（保持你的原值）
        self.wheel_base = 1.072
        self.wheel_radius = 0.125
        self.wheel_steering_y_offset = 0.0
        self.wheel_seperation = 0.615
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        # 参数化的底盘速度增益（将 m/s、rad/s 映射到关节速度）
        # 你原来 joystick 用 factor=10，保留默认 10.0，按需要在 Launch/ROS 参数里覆盖
        self.drive_gain = float(self.declare_parameter('drive_gain', 10.0).value)

        # 初始化
        self.pos = np.array([0.0, 0.0, 0.0, 0.0], float)  # four steering angles
        self.vel = np.array([0.0, 0.0, 0.0, 0.0], float)  # four wheel speeds
        self.ra_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], float)

        # 底盘速度（来自 /cmd_vel）
        self.base_cmd = Twist()

        # 发布器
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_ra_pos = self.create_publisher(Float64MultiArray, '/robotic_arm_controller/commands', 10)

        # 订阅仲裁后的底盘 cmd_vel（twist_mux 输出）
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 定时器
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 死区（对底盘 /cmd_vel 做轻微滤波，避免“爬行”）
        self.deadzone = 0.05

    # /cmd_vel 回调：仅更新底盘速度
    def cmd_vel_cb(self, msg: Twist):
        self.base_cmd = msg

    def timer_callback(self):
        global vel_msg, mode_selection

        # 仅在 mode=4（驾驶）时根据 /cmd_vel 计算底盘
        if mode_selection == 4:
            vx = self.base_cmd.linear.x
            vy = self.base_cmd.linear.y
            wz = self.base_cmd.angular.z

            # 简单死区抑制
            if abs(vx) < self.deadzone: vx = 0.0
            if abs(vy) < self.deadzone: vy = 0.0
            if abs(wz) < self.deadzone: wz = 0.0

            if wz != 0.0:
                if vx != 0.0:
                    # ---- Ackermann 混动（转弯+前进）----
                    factor = self.drive_gain

                    # 避免 0 除
                    if abs(wz) < 1e-5:
                        self.pos[:] = 0.0
                    else:
                        r = abs(vx) / wz * 2*math.pi
                        r_bl = r + self.steering_track / 2.0
                        r_br = r - self.steering_track / 2.0

                        a_fl = math.atan(self.wheel_base / r_bl)
                        a_fr = math.atan(self.wheel_base / r_br)

                        # 当旋转中心落在两轮之间时的 >90° 修正
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
                    # ---- 原地转向 Pivot Turn ----
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
                # ---- Crab Walk（横移/斜移）----
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

        # -----------------------
        # 机械臂控制（保持你的原逻辑，使用 vel_msg 作为输入）
        # -----------------------
        if mode_selection == 5:
            if int(vel_msg.linear.y) != 0:
                command = self.ra_pos[0] - vel_msg.linear.y * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[0] = command
                else:
                    if np.sign(vel_msg.linear.y)*np.sign(self.ra_pos[0]) < 0:
                        self.ra_pos[0] = np.sign(self.ra_pos[0]) * 1.5
                    else:
                        self.ra_pos[0] = command

        elif mode_selection == 6:
            if int(vel_msg.linear.x) != 0:
                command = self.ra_pos[1] - vel_msg.linear.x * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[1] = command
                else:
                    if np.sign(vel_msg.linear.x)*np.sign(self.ra_pos[1]) < 0:
                        self.ra_pos[1] = np.sign(self.ra_pos[1]) * 1.5
                    else:
                        self.ra_pos[1] = command

        elif mode_selection == 7:
            if int(vel_msg.linear.x) != 0:
                command = self.ra_pos[2] - vel_msg.linear.x * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[2] = command
                else:
                    if np.sign(vel_msg.linear.x)*np.sign(self.ra_pos[2]) < 0:
                        self.ra_pos[2] = np.sign(self.ra_pos[2]) * 1.5
                    else:
                        self.ra_pos[2] = command

        elif mode_selection == 8:
            if int(vel_msg.angular.y) != 0:
                command = self.ra_pos[3] - vel_msg.angular.y * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[3] = command
                else:
                    if np.sign(vel_msg.angular.y)*np.sign(self.ra_pos[3]) < 0:
                        self.ra_pos[3] = np.sign(self.ra_pos[3]) * 1.5
                    else:
                        self.ra_pos[3] = command

        elif mode_selection == 9:
            if int(vel_msg.angular.z) != 0:
                command = self.ra_pos[4] - vel_msg.angular.z * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[4] = command
                else:
                    if np.sign(vel_msg.angular.z)*np.sign(self.ra_pos[4]) < 0:
                        self.ra_pos[4] = np.sign(self.ra_pos[4]) * 1.5
                    else:
                        self.ra_pos[4] = command

        elif mode_selection == 10:
            if int(vel_msg.angular.z) != 0:
                command = self.ra_pos[5] - vel_msg.angular.z * 0.005
                if abs(command) < 1.5:
                    self.ra_pos[5] = command
                else:
                    if np.sign(vel_msg.angular.z)*np.sign(self.ra_pos[5]) < 0:
                        self.ra_pos[5] = np.sign(self.ra_pos[5]) * 1.5
                    else:
                        self.ra_pos[5] = command

        if mode_selection == 11:
            if round(vel_msg.angular.z, 3) != 0:
                command = self.ra_pos[6] - vel_msg.angular.z * 0.005
                if command >= 0.0:
                    if abs(command) < 0.29:
                        self.ra_pos[6] = command
                    else:
                        if np.sign(vel_msg.angular.z)*np.sign(self.ra_pos[6]) < 0:
                            self.ra_pos[6] = np.sign(self.ra_pos[6]) * 0.29
                        else:
                            self.ra_pos[6] = command
                else:
                    self.ra_pos[6] = 0.0

        # 发布
        self.pub_pos.publish(Float64MultiArray(data=self.pos))
        self.pub_vel.publish(Float64MultiArray(data=self.vel))
        self.pub_ra_pos.publish(Float64MultiArray(data=self.ra_pos))

        # 清空底盘瞬时命令（下一周期重算）
        self.pos[:] = 0.0
        self.vel[:] = 0.0
        # 机械臂位置保持不清零（ra_pos 是位置目标）

class Joy_subscriber(Node):
    """ 仅负责读取手柄，在不同按钮下切换 mode_selection，并把轴值写入 vel_msg（供机械臂控制使用） """
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)

    def listener_callback(self, data: Joy):
        global vel_msg, mode_selection
        if not data.axes:
            return

        # 模式选择（按钮/方向键）
        if data.buttons[1] == 1:       # B -> robotic arm base
            mode_selection = 5
        elif data.buttons[3] == 1:     # Y -> shoulder
            mode_selection = 6
        elif data.buttons[2] == 1:     # X -> elbow
            mode_selection = 7
        elif data.axes[6] == -1:       # d-pad right -> wrist_1
            mode_selection = 8
        elif data.axes[7] == 1:        # d-pad up -> wrist_2
            mode_selection = 9
        elif data.axes[6] == 1:        # d-pad left -> wrist_3
            mode_selection = 10
        elif data.axes[7] == -1:       # d-pad down -> gripper
            mode_selection = 11
        else:
            # 默认回到驾驶模式（底盘从 /cmd_vel 获得速度）
            mode_selection = 4

        # 把手柄轴值写入 vel_msg（仅供机械臂使用）
        vel_msg.linear.x  = data.axes[1]
        vel_msg.linear.y  = data.axes[0]
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

    try:
        # rclpy 没有 create_rate on Node（此前那行会抛错），用 sleep 循环就行
        while rclpy.ok():
            rclpy.spin_once(robot_control, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
