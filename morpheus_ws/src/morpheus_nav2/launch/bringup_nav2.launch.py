from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    cmd_vel_topic  = LaunchConfiguration('cmd_vel_topic',  default='/cmd_vel_nav')

    mode           = LaunchConfiguration('mode',           default='localization')
    slam_params    = LaunchConfiguration('slam_params',    default=os.path.join(pkg, 'config', 'slam_toolbox.yaml'))

    with_teleop    = LaunchConfiguration('with_teleop',    default='true')
    with_joy       = LaunchConfiguration('with_joy',       default='false')
    teleop_cfg     = LaunchConfiguration('teleop_cfg',     default=os.path.join(pkg, 'config', 'teleop_joy.yaml'))
    twist_mux_cfg  = LaunchConfiguration('twist_mux_cfg',  default=os.path.join(pkg, 'config', 'twist_mux.yaml'))

    is_localization = PythonExpression(["'", mode, "' == 'localization'"])
    is_mapping      = PythonExpression(["'", mode, "' == 'mapping'"])

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
        description='Where controller_server/behavior_server publish cmd_vel (feeds twist_mux)'
    )
    declare_mode = DeclareLaunchArgument(
        'mode', default_value=mode,
        description="'localization' (AMCL + map_server) or 'mapping' (slam_toolbox online SLAM)"
    )
    declare_slam_params = DeclareLaunchArgument(
        'slam_params', default_value=slam_params,
        description='Full path to slam_toolbox YAML config'
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
    # Point cloud ground segmentation
    # ---------------------------
    ground_seg = Node(
        package='morpheus_nav2',
        executable='ground_segmentation.py',
        name='ground_segmentation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------------------
    # Nav2 Nodes
    # ---------------------------

    # --- Localization mode: map_server + AMCL ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml}],
        condition=IfCondition(is_localization)
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(is_localization)
    )

    # --- Mapping mode: slam_toolbox online async ---
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(is_mapping)
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # 关键①：把 controller_server 的 /cmd_vel 重映射到 /cmd_vel_nav
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

    # 关键②：behavior_server 也会发布 /cmd_vel（多个插件各占一个 publisher）
    # 同样重映射到 /cmd_vel_nav，杜绝直接写 /cmd_vel
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', cmd_vel_topic)]
    )

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    lifecycle_mgr_loc = Node(
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
                'bt_navigator',
                'waypoint_follower'
            ]
        }],
        condition=IfCondition(is_localization)
    )

    lifecycle_mgr_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }],
        condition=IfCondition(is_mapping)
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
        declare_mode,
        declare_slam_params,
        declare_with_teleop,
        declare_with_joy,
        declare_teleop_cfg,
        declare_twist_mux_cfg,

        # Ground segmentation (feeds voxel_layer)
        ground_seg,

        # Localization mode
        map_server,
        amcl,
        # Mapping mode
        slam_toolbox,
        # Shared Nav2 nodes
        planner,
        controller,
        smoother,
        behavior,
        bt_nav,
        waypoint_follower,
        lifecycle_mgr_loc,
        lifecycle_mgr_map,

        # teleop + mux
        joy_node,
        teleop,
        twist_mux,
    ])
