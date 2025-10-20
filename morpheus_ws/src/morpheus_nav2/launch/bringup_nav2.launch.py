from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg = get_package_share_directory('morpheus_nav2')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # map_server: 建议在 nav2_params.yaml 里配置 yaml_filename/path 等
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
    )

    # 关键：把 Nav2 的 /cmd_vel 改成 /cmd_vel_nav，供 twist_mux 仲裁
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
    )

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params],
    )

    # 可选：若你在 params 启用了 waypoint_follower，这里也可以一起带上
    # waypoint_follower = Node(
    #     package='nav2_waypoint_follower',
    #     executable='waypoint_follower',
    #     name='waypoint_follower',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}, params],
    # )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,   # 避免仿真中 bond 抖动引发反复重启
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                # 'waypoint_follower',  # 如启用则取消注释
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (Gazebo)'),
        map_server,
        amcl,
        planner,
        controller,
        smoother,
        behavior,
        bt_nav,
        # waypoint_follower,
        lifecycle_mgr,
    ])
