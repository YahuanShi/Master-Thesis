import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    with_visual_odom = LaunchConfiguration('with_visual_odom', default='false')

    # PythonExpression evaluates at runtime so the condition survives
    # being passed through IncludeLaunchDescription from morpheus_spawn
    is_localization = PythonExpression(["'", mode, "' == 'localization'"])
    is_mapping      = PythonExpression(["'", mode, "' == 'mapping'"])
    is_not_mapping  = PythonExpression(["'", mode, "' != 'mapping'"])

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
    # Static map→odom TF (simulation bypass)
    # ---------------------------
    # In simulation, the EKF fuses Gazebo's ground-truth odometry (/gz/odom),
    # so the odom frame already coincides with the world frame.  The map→odom
    # transform should therefore be the identity.
    # AMCL normally publishes this TF, but on sloped terrain its particle filter
    # diverges (the 3D LiDAR scan doesn't match the flat 2D map), causing the
    # rover to plan paths in the wrong reference frame.
    # Fix: publish a static identity map→odom TF and tell AMCL not to broadcast
    # (tf_broadcast: false in nav2_params.yaml).  AMCL still runs because the
    # lifecycle manager requires it, but it no longer touches the TF tree.
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        condition=IfCondition(is_not_mapping),  # not needed in mapping: slam_toolbox provides the TF
    )

    # ---------------------------
    # Visual odometry (rtabmap rgbd_odometry)
    # Disabled by default (with_visual_odom=false) — high CPU cost in simulation.
    # Enable for real hardware where wheel odometry is unreliable.
    # ---------------------------
    vo_params = os.path.join(pkg, 'config', 'visual_odom.yaml')

    visual_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[vo_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/camera_2i'),
            ('depth/image', '/camera_2i/depth_image'),
            ('rgb/camera_info', '/camera_2i/camera_info'),
            ('odom', '/visual_odom'),
        ],
        condition=IfCondition(with_visual_odom),
    )

    # ---------------------------
    # ArUco marker-based localization correction
    # ---------------------------
    aruco_map_cfg = os.path.join(pkg, 'config', 'aruco_marker_map.yaml')

    aruco_localization = Node(
        package='morpheus_nav2',
        executable='aruco_localization.py',
        name='aruco_localization',
        output='screen',
        parameters=[aruco_map_cfg, {'use_sim_time': use_sim_time}],
    )

    # ---------------------------
    # Diagnostic aggregator
    # ---------------------------
    diag_cfg = os.path.join(pkg, 'config', 'diagnostics.yaml')

    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[diag_cfg, {'use_sim_time': use_sim_time}],
    )

    # ---------------------------
    # Point cloud ground segmentation
    # Subscribes to /scan/points (raw LiDAR) and publishes:
    #   /scan/points/obstacles — above-ground objects + steep slopes
    #   /scan/points/ground    — flat traversable ground
    # Both outputs feed the voxel_layer in the Nav2 costmaps.
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

    # Localization mode: pre-built map + AMCL particle filter
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

    # Mapping mode: slam_toolbox builds the map online while the rover explores
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

    # Remap /cmd_vel → /cmd_vel_nav so Nav2 output goes through twist_mux
    # rather than directly to the robot.  twist_mux merges Nav2 + teleop with
    # priority, ensuring joystick can always override autonomous navigation.
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

    # behavior_server also publishes /cmd_vel (for spin/backup recovery behaviors)
    # — must be remapped to /cmd_vel_nav for the same reason as controller_server
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', cmd_vel_topic)]
    )

    bt_xml = os.path.join(pkg, 'behavior_trees', 'morpheus_nav_to_pose.xml')

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'default_bt_xml_filename': bt_xml,
        }]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Two lifecycle managers: one for localization mode (includes map_server + amcl),
    # one for mapping mode (excludes them — slam_toolbox manages its own lifecycle).
    # Only one runs at a time based on the `mode` argument.
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
        remappings=[('/cmd_vel', '/cmd_vel_joy')],  # separate topic so twist_mux can prioritise
        condition=IfCondition(with_teleop)
    )

    # twist_mux always runs (no with_teleop condition) because Nav2 navigation
    # requires it to bridge /cmd_vel_nav → /cmd_vel.  Without twist_mux running,
    # autonomous navigation produces no motion even when teleop is not in use.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],  # mux output → robot driver topic
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

        # Static map→odom (simulation ground truth bypass)
        static_map_odom,

        # Visual odometry (feeds EKF as odom1)
        visual_odom,

        # ArUco localization correction
        aruco_localization,

        # Diagnostics
        diagnostic_aggregator,

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
