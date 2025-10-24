from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    drive_gain   = LaunchConfiguration('drive_gain',   default='10.0')

    return LaunchDescription([
        # -------- Launch args --------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'drive_gain',
            default_value='10.0',
            description='Scale factor mapping /cmd_vel to wheel joint velocity'
        ),

        # -------- Joystick driver (publishes /joy) --------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # 常用参数，可按需调整
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,      # 按住按钮时重复率 (Hz)
                'coalesce_interval': 0.02,    # 合并事件的时间窗口 (s)
                # 'dev': '/dev/input/js0',    # 如需指定设备，取消注释并设置
                # 'sticky_buttons': False
            }]
        ),

        # -------- Morpheus control (reads /cmd_vel for base, /joy for arm) --------
        Node(
            package='morpheus_control',
            executable='morpheus_control.py',
            name='morpheus_control',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'drive_gain': drive_gain
            }]
            # 这里不需要 remap /cmd_vel：
            # twist_mux（或 Nav2）应输出到 /cmd_vel，本节点直接订阅即可
        ),
    ])
