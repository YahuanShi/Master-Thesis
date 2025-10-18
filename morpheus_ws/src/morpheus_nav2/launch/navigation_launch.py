import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 nav2_bringup 包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 定义仿真时间参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 定义 Nav2 配置文件的路径 (我们稍后会创建这些文件)
    nav2_params_file = os.path.join(
        get_package_share_directory('morpheus_nav2'),
        'config',
        'nav2_params.yaml'
    )

    # --- 1. SLAM (Simultaneous Localization and Mapping) 节点 ---
    # 使用 SLAM Toolbox 进行建图和定位
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file, # 这里我们将使用一个统一的配置文件
            'autostart': 'true'
        }.items(),
    )

    # --- 2. 导航堆栈核心 (Planner, Controller, Recoveries) ---
    # 启动 Nav2 导航堆栈
    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            # 'map_subscribe_transient_default': 'true' # 仅用于 map server
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # slam_toolbox_node,  # 暂时禁用 SLAM，先跑基础导航
        nav_stack,          # 启动导航核心
    ])