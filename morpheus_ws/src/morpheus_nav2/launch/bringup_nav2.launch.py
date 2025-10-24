from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('morpheus_nav2')

    # ---------------------------
    # Launch args
    # ---------------------------
    use_sim_time   = LaunchConfiguration('use_sim_time',   default='true')
    params_file    = LaunchConfiguration('params_file',    default=os.path.join(pkg, 'config', 'nav2_params.yaml'))
    map_yaml       = LaunchConfiguration('map',            default=os.path.join(pkg, 'config', 'map.yaml'))
    autostart      = LaunchConfiguration('autostart',      default='true')
    # 让 Nav2 的速度输出到 /cmd_vel_nav，供 twist_mux 仲裁
    cmd_vel_topic  = LaunchConfiguration('cmd_vel_topic',  default='/cmd_vel_nav')

    # teleop_twist_joy + twist_mux（可选）
    with_teleop    = LaunchConfiguration('with_teleop',    default='true')
    with_joy       = LaunchConfiguration('with_joy',       default='false')  # 若你的 joy_node 在别处已启动，保持 false
    teleop_cfg     = LaunchConfiguration('teleop_cfg',     default=os.path.join(pkg, 'config', 'teleop_joy.yaml'))
    twist_mux_cfg  = LaunchConfiguration('twist_mux_cfg',  default=os.path.join(pkg, 'config', 'twist_mux.yaml'))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=use_sim_time,
        description='Use simulation (Gazebo) clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=params_file,
        description='Full path to nav2_params.yaml'
    )
    declare_map_yaml = DeclareLaunchArgument(
        'map', default_value=map_yaml,
        description='Full path to map.yaml'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value=autostart,
        description='Automatically startup the Nav2 stack'
    )
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=cmd_vel_topic,
        description='Where controller_server publishes cmd_vel (feed into twist_mux)'
    )

    declare_with_teleop = DeclareLaunchArgument(
        'with_teleop', default_value=with_teleop,
        description='Launch teleop_twist_joy and twist_mux'
    )
    declare_with_joy = DeclareLaunchArgument(
        'with_joy', default_value=with_joy,
        description='Also launch joy_node (only if not launched elsewhere)'
    )
    declare_teleop_cfg = DeclareLaunchArgument(
        'teleop_cfg', default_value=teleop_cfg,
        description='teleop_twist_joy YAML config'
    )
    declare_twist_mux_cfg = DeclareLaunchArgument(
        'twist_mux_cfg', default_value=twist_mux_cfg,
        description='twist_mux YAML config'
    )

    # ---------------------------
    # Nav2 Nodes
    # ---------------------------

    # map_server 既读 params_file，也接受显式 map.yaml 覆盖（更稳）
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 关键：把 /cmd_vel 重映射到 /cmd_vel_nav，以便与 twist_mux 对接
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', cmd_vel_topic)]
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 可选：若用 waypoint_follower，可在 nav2_params.yaml 配置后放开
    # waypoint_follower = Node(
    #     package='nav2_waypoint_follower',
    #     executable='waypoint_follower',
    #     name='waypoint_follower',
    #     output='screen',
    #     parameters=[params_file, {'use_sim_time': use_sim_time}]
    # )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator'
                # 'waypoint_follower'  # 如启用则加入
            ]
        }]
    )

    # ---------------------------
    # teleop_twist_joy + twist_mux
    # ---------------------------

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(with_joy)
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[teleop_cfg, {'use_sim_time': use_sim_time}],
        # 双保险：即使 YAML 里已设置 cmd_vel_topic，这里也统一改名
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        condition=IfCondition(with_teleop)
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg, {'use_sim_time': use_sim_time}],
        # mux 输出到标准 /cmd_vel，供 morpheus_control 订阅
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        condition=IfCondition(with_teleop)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map_yaml,
        declare_autostart,
        declare_cmd_vel_topic,
        declare_with_teleop,
        declare_with_joy,
        declare_teleop_cfg,
        declare_twist_mux_cfg,

        # Nav2
        map_server,
        amcl,
        planner,
        controller,
        smoother,
        behavior,
        bt_nav,
        # waypoint_follower,
        lifecycle_mgr,

        # teleop + mux
        joy_node,
        teleop,
        twist_mux,
    ])
