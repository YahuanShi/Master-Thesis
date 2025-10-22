# morpheus_spawn_launch.py

from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
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
    # ArUco recognition (uses zed_2i)
    # ----------------------------
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
            '-x',
            '0.0',
            '-y',
            '4.0',
            '-z',
            '1.5',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
        ],
    )

    # ----------------------------
    # Controllers
    # ----------------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morpheus_control_path, 'launch', 'morpheus_control.launch.py')
        ),
    )
    
    # load_ackermann_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'test_ackermann_steering_controller'],
    #     output='screen'
    # )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )

    load_forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'],
        output='screen',
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'],
        output='screen',
    )

    load_suspension_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'suspension_controller'],
        output='screen',
    )

    load_robotic_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robotic_arm_controller'],
        output='screen',
    )

    activate_joint_state_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    
    # load_gripper_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'robotic_gripper_controller'],
    #     output='screen'
    # )

    activate_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[
                load_forward_velocity_controller,
                load_forward_position_controller,
                load_suspension_controller,
                load_robotic_arm_controller,
            ],
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
    # # Mini camera (Robotic arm)
    # bridge_camera_mini = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/camera_mini@sensor_msgs/msg/Image@gz.msgs.Image'], 
    #     output='screen'
    # ) 

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
    return LaunchDescription(
        [
            gazebo_resource_path,
            arguments,
            gazebo,
            node_robot_state_publisher,
            gz_spawn_entity,
            activate_joint_state_controller,
            activate_controllers,
            controller,
            activate_nav2,
            bridge_camera_2i,
            bridge_camera_2i_info,
            bridge_scan,
            bridge_imu,
            bridge_cloud,
            aruco_node,
            # bridge_camera_mini, # 机械臂摄像头，非导航核心，可选启用
            # bridge_camera_info,,
            # rviz,  # 可按需启用
        ]
    )
