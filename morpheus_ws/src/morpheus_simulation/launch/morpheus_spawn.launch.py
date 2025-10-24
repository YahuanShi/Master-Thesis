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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ----------------------------
    # Launch Arguments
    # ----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # ----------------------------
    # Package Paths
    # ----------------------------
    morpheus_control_path = get_package_share_directory('morpheus_control')
    morpheus_description_path = get_package_share_directory('morpheus_description')
    morpheus_simulation_path = get_package_share_directory('morpheus_simulation')
    morpheus_nav2_path = get_package_share_directory('morpheus_nav2')

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
        launch_arguments=[
            (
                'gz_args',
                [
                    LaunchConfiguration('world'),
                    '.sdf',
                    ' -v 1',
                    ' -r',
                    ' -s'
                ],
            )
        ],
    )

    arguments = LaunchDescription(
        [DeclareLaunchArgument('world', default_value='marsyard2022', description='GZ Sim world')]
    )

    # ----------------------------
    # (Optional) ArUco recognition (uses zed_2i)
    # - 用 try/except 防止未安装时阻塞启动
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
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        )
    except Exception:
        # 未安装 ros2_aruco 时跳过
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
        # NOTE: do NOT remap /tf or /tf_static or Nav2/AMCL won't see TFs
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_desc,
            '-x', '0.0',
            '-y', '4.0',
            '-z', '1.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    # ----------------------------
    # Controllers via spawner
    # ----------------------------
    # 注意：如果你在 morpheus_control.launch.py 里也会加载控制器，请确保不要重复加载
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morpheus_control_path, 'launch', 'morpheus_control.launch.py')
        ),
    )

    CONTROLLER_MGR = '/controller_manager'  # gz_ros2_control 默认名
    
    spawner_ackermann = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_ackermann_controller',
        arguments=['ackermann_controller', '--controller-manager', CONTROLLER_MGR],
        output='screen'
    )

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
    
    # spawner_gripper = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     name='spawner_gripper_controller',
    #     arguments=['gripper_controller', '--controller-manager', CONTROLLER_MGR],
    #     output='screen'
    # )

    # 顺序：spawn 完成 → 1s 后 JSB → JSB 成功后并行加载其它
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
    # Nav2 Bringup
    # ----------------------------
    activate_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morpheus_nav2_path, 'launch', 'bringup_nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ----------------------------
    # ROS ⇄ Gazebo bridges
    # ----------------------------
    # Lidar scan
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen',
    )

    # Lidar points
    bridge_cloud = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen',
    )

    # Main camera (zed_2i)
    bridge_camera_2i = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_2i@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
    )

    # Main camera info (zed_2i)
    bridge_camera_2i_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_2i/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen',
    )

    # Mini camera (Robotic arm)
    bridge_camera_mini = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_mini@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Gazebo odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/marsyard2022/model/morpheus_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen',
    )

    # EKF（发布 odom->base_link）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf',
        output='screen',
        parameters=[os.path.join(morpheus_nav2_path, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    # IMU
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen',
    )

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
    )

    # ----------------------------
    # LaunchDescription
    # ----------------------------
    ld_items = [
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        # base_link 别名（需要的话）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_alias',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'chassis_link', 'base_link'],
            output='screen'
        ),
        gz_spawn_entity,

        # 控制器：事件式顺序
        activate_jsb_after_spawn,
        activate_rest_after_jsb,

        controller,          # 注意避免在这个 include 内部重复加载控制器
        activate_nav2,

        # Bridges & sensors
        bridge_camera_2i,
        bridge_camera_2i_info,
        bridge_scan,
        bridge_odom,
        ekf_node,
        bridge_imu,
        bridge_cloud,

        # 可选
        # aruco_node,
        # bridge_camera_mini,
        # bridge_camera_info,
        # rviz,
    ]

    if aruco_node is not None:
        ld_items.append(aruco_node)

    return LaunchDescription(ld_items)
