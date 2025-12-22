from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------
    # Launch Arguments
    # ----------------------------
    world_arg      = LaunchConfiguration('world')
    gui_arg        = LaunchConfiguration('gui')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    nav_mode       = LaunchConfiguration('mode')
    with_rviz      = LaunchConfiguration('with_rviz')
    with_teleop    = LaunchConfiguration('with_teleop')

    spawn_x   = LaunchConfiguration('spawn_x')
    spawn_y   = LaunchConfiguration('spawn_y')
    spawn_z   = LaunchConfiguration('spawn_z')
    spawn_R   = LaunchConfiguration('spawn_R')
    spawn_P   = LaunchConfiguration('spawn_P')
    spawn_Y   = LaunchConfiguration('spawn_Y')

    declare_args = [
        DeclareLaunchArgument('world',        default_value='marsyard2022',
                              description='GZ Sim world name (without .sdf)'),
        DeclareLaunchArgument('gui',          default_value='true',
                              description='Start Gazebo GUI'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo clock'),
        DeclareLaunchArgument('mode',         default_value='localization',
                              description="Nav2: 'localization' (AMCL) or 'mapping' (slam_toolbox)"),
        DeclareLaunchArgument('with_rviz',    default_value='false',
                              description='Launch RViz2'),
        DeclareLaunchArgument('with_teleop',  default_value='true',
                              description='Launch teleop_twist_joy + twist_mux'),
        DeclareLaunchArgument('spawn_x',      default_value='0.0',
                              description='Rover spawn X position'),
        DeclareLaunchArgument('spawn_y',      default_value='4.0',
                              description='Rover spawn Y position'),
        DeclareLaunchArgument('spawn_z',      default_value='1.5',
                              description='Rover spawn Z position'),
        DeclareLaunchArgument('spawn_R',      default_value='0.0',
                              description='Rover spawn roll'),
        DeclareLaunchArgument('spawn_P',      default_value='0.0',
                              description='Rover spawn pitch'),
        DeclareLaunchArgument('spawn_Y',      default_value='0.0',
                              description='Rover spawn yaw'),
    ]

    # ----------------------------
    # Package Paths
    # ----------------------------
    morpheus_control_path     = get_package_share_directory('morpheus_control')
    morpheus_description_path = get_package_share_directory('morpheus_description')
    morpheus_simulation_path  = get_package_share_directory('morpheus_simulation')
    morpheus_nav2_path        = get_package_share_directory('morpheus_nav2')

    # ----------------------------
    # Gazebo: resources + launcher
    # ----------------------------
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(morpheus_simulation_path, 'worlds'),
            ':' + str(Path(morpheus_description_path).parent.resolve()),
        ],
    )

    gz_pkg_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_pkg_path, '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': [world_arg, '.sdf', ' -v 1', ' -r'],
            'gui': gui_arg,
        }.items(),
    )

    # ----------------------------
    # (Optional) ArUco recognition (uses zed_2i)
    # ----------------------------
    aruco_node = None
    try:
        aruco_params = os.path.join(
            get_package_share_directory('ros2_aruco'),
            'config',
            'aruco_parameters.yaml',
        )
        aruco_node = Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_detector_2i',
            parameters=[
                aruco_params,
                {'image_topic': '/camera_2i'},
                {'camera_info_topic': '/camera_2i/camera_info'},
                {'camera_frame': 'zed_2i_link'},
                {'marker_size': 0.1},
                {'aruco_dictionary_id': 'DICT_5X5_50'},
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        )
    except Exception:
        aruco_node = None

    # ----------------------------
    # Robot spawn (xacro -> URDF)
    # ----------------------------
    xacro_file = os.path.join(morpheus_description_path, 'urdf', 'robot.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time},
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-R', spawn_R,
            '-P', spawn_P,
            '-Y', spawn_Y,
        ],
    )

    static_base_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_alias',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'chassis_link', 'base_link'],
        output='screen'
    )

    # ----------------------------
    # Controllers via spawner
    # ----------------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morpheus_control_path, 'launch', 'morpheus_control.launch.py')
        ),
    )

    CONTROLLER_MGR = '/controller_manager'

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', CONTROLLER_MGR],
        output='screen'
    )

    spawner_vel = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_forward_velocity_controller',
        arguments=['forward_velocity_controller', '--controller-manager', CONTROLLER_MGR],
        output='screen'
    )

    spawner_pos = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_forward_position_controller',
        arguments=['forward_position_controller', '--controller-manager', CONTROLLER_MGR],
        output='screen'
    )

    spawner_susp = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_suspension_controller',
        arguments=['suspension_controller', '--controller-manager', CONTROLLER_MGR],
        output='screen'
    )

    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robotic_arm_controller',
        arguments=['robotic_arm_controller', '--controller-manager', CONTROLLER_MGR],
        output='screen'
    )

    activate_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[TimerAction(period=1.0, actions=[spawner_jsb])],
        )
    )

    activate_rest_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_vel, spawner_pos, spawner_susp, spawner_arm],
        )
    )

    # ----------------------------
    # ROS-Gazebo bridges
    # ----------------------------
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen',
    )

    bridge_cloud = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cloud',
        arguments=['/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen',
    )

    bridge_camera_2i = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_2i',
        arguments=['/camera_2i@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
    )

    bridge_camera_2i_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_2i_info',
        arguments=['/camera_2i/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen',
    )

    bridge_depth_2i = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_2i',
        arguments=['/camera_2i/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
    )

    bridge_depth_cloud_2i = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_cloud_2i',
        arguments=['/camera_2i/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen',
    )

    bridge_camera_mini = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_mini',
        arguments=['/camera_mini@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Odom bridge — world name is parameterized; republish to /gz/odom
    # so EKF config doesn't need to know the world name
    odom_bridge_arg = PythonExpression([
        "'/world/' + '", world_arg,
        "' + '/model/morpheus_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'"
    ])
    odom_remap_src = PythonExpression([
        "'/world/' + '", world_arg,
        "' + '/model/morpheus_rover/odometry'"
    ])
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=[odom_bridge_arg],
        remappings=[(odom_remap_src, '/gz/odom')],
        output='screen',
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen',
    )

    # ----------------------------
    # EKF (publish odom->base_link)
    # ----------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf',
        output='screen',
        parameters=[os.path.join(morpheus_nav2_path, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # ----------------------------
    # Nav2 Bringup
    # ----------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morpheus_nav2_path, 'launch', 'bringup_nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'with_teleop': with_teleop,
            'mode': nav_mode,
        }.items(),
    )
    activate_nav2 = TimerAction(period=2.0, actions=[nav2_bringup_launch])

    # ----------------------------
    # RViz (optional)
    # ----------------------------
    rviz_config_file = os.path.join(morpheus_simulation_path, 'launch', 'morpheus_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(with_rviz),
    )

    # ----------------------------
    # LaunchDescription
    # ----------------------------
    ld_items = [
        *declare_args,
        gazebo_resource_path,
        gazebo,

        node_robot_state_publisher,
        static_base_alias,
        gz_spawn_entity,

        activate_jsb_after_spawn,
        activate_rest_after_jsb,

        controller,

        bridge_clock,
        bridge_camera_2i,
        bridge_camera_2i_info,
        bridge_scan,
        bridge_odom,
        bridge_imu,
        bridge_cloud,
        bridge_depth_2i,
        bridge_depth_cloud_2i,

        ekf_node,
        activate_nav2,

        rviz,
    ]

    if aruco_node is not None:
        ld_items.append(aruco_node)

    return LaunchDescription(ld_items)
