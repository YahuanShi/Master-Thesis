#!/usr/bin/python3
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray 
from rclpy.qos import qos_profile_system_default # 导入默认 QoS

class Nav2_Kinematics_Controller(Node):

    def __init__(self):
        super().__init__('nav2_kinematics_controller')

        # 设置控制循环频率 (与 YAML 中的 update_rate: 100 匹配)
        self.timer_period = 0.01  # 100 Hz

        # -----------------------------------------------------------
        # 机器人几何参数 (来自 morpheus_control.py)
        # -----------------------------------------------------------
        self.wheel_base = 1.072           # L: 前后轴距
        self.wheel_radius = 0.125
        self.wheel_steering_y_offset = 0.0 # y_offset: 转向机构在 Y 轴的偏移 (这里为 0)
        self.wheel_seperation = 0.615     # W: 左右轮间距
        self.steering_track = self.wheel_seperation - 2 * self.wheel_steering_y_offset # W_eff

        # -----------------------------------------------------------
        # 初始化状态变量
        # -----------------------------------------------------------
        # 转向关节位置指令 (steering_fl/fr/rl/rr_joint)
        self.pos = np.array([0, 0, 0, 0], float) 
        # 驱动轮速度指令 (wheel_fl/fr/rl/rr_joint)
        self.vel = np.array([0, 0, 0, 0], float) 
        # 用于存储 Nav2 的 Twist 指令
        self.cmd_vel_msg = Twist()

        # -----------------------------------------------------------
        # ROS 2 通信设置
        # -----------------------------------------------------------
        
        # 订阅 Nav2 发布的速度指令 (来自 /cmd_vel)
        self.create_subscription(
            Twist, 
            'cmd_vel',                 # Nav2 标准输出话题
            self.cmd_vel_callback,     
            qos_profile_system_default # 使用默认的系统 QoS
        )

        # 定义控制器指令发布器 (URDF 中配置的控制器)
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # 启动控制循环定时器
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def cmd_vel_callback(self, msg):
        """接收来自 Nav2 的 Twist 指令并存储"""
        self.cmd_vel_msg = msg

    def timer_callback(self):
        """主控制循环：执行逆运动学并发布关节指令"""
        
        # 从 Nav2 指令中获取速度分量
        linear_x = self.cmd_vel_msg.linear.x
        linear_y = self.cmd_vel_msg.linear.y
        angular_z = self.cmd_vel_msg.angular.z

        # 重置指令数组
        self.pos[:] = 0.0
        self.vel[:] = 0.0
        
        # -----------------------------------------------------------
        # 运动模式判断和逆运动学计算
        # -----------------------------------------------------------
        
        # 移除原 morpheus_control.py 中的 Factor = 10，因为 Nav2 发布的是目标物理速度
        # 假设 Nav2 发布的指令已经是最终的物理值 (m/s 和 rad/s)
        
        # 1. Ackermann 模式 (转向 + 前进/后退)
        if abs(angular_z) > 1e-5 and abs(linear_x) > 1e-5:
            
            # --- 转向角度计算 ---
            # r 的定义应为曲率半径的倒数，此处 r=Vx/Wz 看起来是 Nav2 默认的定义方式
            # r 在原代码中的定义: r = |Vx| / Wz * 2*math.pi (这与标准曲率半径定义不同，但我们保留原代码的公式结构)
            r = abs(linear_x) / angular_z * 2 * math.pi
            r_bl = r + self.steering_track / 2  # 左侧转弯半径
            r_br = r - self.steering_track / 2  # 右侧转弯半径

            # 计算转向角度 (前轮)
            a_fl = math.atan(self.wheel_base / r_bl)
            a_fr = math.atan(self.wheel_base / r_br)
            
            # (原代码中的 >90 度转向调整，保留原逻辑)
            if r_bl > 0 and r < 0:
                a_fl -= math.pi
            if r_br < 0 and r > 0:
                a_fr += math.pi

            # 四个转向关节的指令 (前轮转向，后轮保持直行)
            # 注意: Morpheus 是四轮转向（Ackermann），但原代码只计算了前轮角度
            # 我们假设后轮角度为 0，因为 Ackermann 转向主要在前轮。
            self.pos[0] = a_fl * 1.57 # 前左轮
            self.pos[1] = a_fr * 1.57 # 前右轮
            self.pos[2] = 0.0         # 后左轮
            self.pos[3] = 0.0         # 后右轮
            
            # --- 轮速计算 ---
            # (原代码的轮速公式非常复杂且似乎是近似，我们简化为标准差速轮速或保持原结构)
            vel_steerring_offset = angular_z * self.wheel_steering_y_offset
            sign = np.sign(linear_x)

            self.vel[0] = sign * math.hypot(
                linear_x - angular_z * self.steering_track / 2,
                angular_z * self.wheel_base / 2) - vel_steerring_offset
            self.vel[1] = sign * math.hypot(
                linear_x + angular_z * self.steering_track / 2,
                angular_z * self.wheel_base / 2) + vel_steerring_offset
            self.vel[2] = sign * math.hypot(
                linear_x - angular_z * self.steering_track / 2,
                angular_z * self.wheel_base / 2) - vel_steerring_offset
            self.vel[3] = sign * math.hypot(
                linear_x + angular_z * self.steering_track / 2,
                angular_z * self.wheel_base / 2) + vel_steerring_offset
            
            # 转换为角速度 (rad/s)
            self.vel[:] /= self.wheel_radius

        # 2. Pivot Turn 模式 (原地旋转)
        elif abs(angular_z) > 1e-5 and abs(linear_x) < 1e-5 and abs(linear_y) < 1e-5:
            
            # 转向角度
            # 45度角（或根据几何尺寸 atan(L/W)）
            pos_angle = math.atan(self.wheel_base / self.steering_track)
            
            # 前左和后右反向，前右和后左反向
            self.pos[0] = -pos_angle
            self.pos[1] = pos_angle
            self.pos[2] = pos_angle
            self.pos[3] = -pos_angle

            # 轮速 (与角速度成正比)
            # 假设 Wz 是机器人中心角速度，轮速 V = Wz * R_pivot，其中 R_pivot = hypot(L/2, W/2)
            R_pivot = math.hypot(self.wheel_base / 2, self.steering_track / 2)
            
            self.vel[0] = -angular_z * R_pivot / self.wheel_radius
            self.vel[1] = angular_z * R_pivot / self.wheel_radius 
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]
            
        # 3. Crab Walk 模式 (纯平移或斜向平移)
        elif abs(linear_x) > 1e-5 or abs(linear_y) > 1e-5:
            
            # 转向角度 (所有轮子角度相同)
            angle = math.atan2(linear_y, linear_x)
            
            # 限制角度在 [-pi/2, pi/2] 之间 (如果需要)
            # if abs(angle) >= math.pi / 2:
            #     angle = (-1) * np.sign(angle) * (math.pi - abs(angle))
                
            self.pos[:] = angle

            # 轮速 (所有轮子速度相同)
            magnitude = math.sqrt(linear_x**2 + linear_y**2)
            # 转换为角速度 (rad/s)
            self.vel[:] = magnitude / self.wheel_radius

        # 4. 停止
        else:
            self.pos[:] = 0.0
            self.vel[:] = 0.0

        # -----------------------------------------------------------
        # 发布指令
        # -----------------------------------------------------------
        
        # 发布转向位置指令
        pos_array = Float64MultiArray(data=self.pos)
        self.pub_pos.publish(pos_array)
        
        # 发布驱动轮速度指令
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_vel.publish(vel_array)


def main():
    rclpy.init()
    node = Nav2_Kinematics_Controller()

    # 使用多线程执行器 (MultiThreadedExecutor) 来确保订阅 (cmd_vel) 
    # 和定时器回调 (timer_callback) 都能及时执行。
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # 保持主线程存活
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()