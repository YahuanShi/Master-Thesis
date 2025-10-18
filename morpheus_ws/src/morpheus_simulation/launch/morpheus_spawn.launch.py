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

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Path Definitions
    morpheus_control_path = get_package_share_directory('morpheus_control')

    morpheus_description_path = get_package_share_directory('morpheus_description')

    morpheus_simulation_path = get_package_share_directory('morpheus_simulation')

    morpheus_nav2_path = get_package_share_directory('morpheus_nav2')
    
    # --------------------------------------------------------------------------------------------------------------
    # Gazebo Launcher:
    # --------------------------------------------------------------------------------------------------------------
    
    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(morpheus_simulation_path, 'worlds'), ':' +
            str(Path(morpheus_description_path).parent.resolve())
            ]
        )

    gz_pkg_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gz_pkg_path, '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 1',
                                 ' -r']
                    )
                ]
             )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='marsyard2022', description='Gz sim World')])

    # ------------------------------------------------------------------------------------------------------------
    # ArUco Recognition Node:
    # ------------------------------------------------------------------------------------------------------------
    
    # 获取 ArUco 配置文件路径 (保持不变)
    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters.yaml'
    )
    
    # 启动 ArUco 节点，并重载其参数以匹配主摄像头 (zed_2i)
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_detector_2i',  # 明确节点名称
        parameters=[aruco_params,
                    # 重载参数，指向主摄像头话题
                    {'image_topic': '/camera_2i'},
                    {'camera_info_topic': '/camera_2i/camera_info'},
                    # 建议设置 camera_frame 以确保 TF 变换正确
                    {'camera_frame': 'zed_2i_link'} 
                   ],
        # 话题重映射：如果 /camera_2i 话题未发布 /image_raw 和 /camera_info
        # 则需要在桥接处设置正确的命名规则，或者在这里进行重映射
        remappings=[
            # 由于 camera_info_topic 可能是 /camera_info，而 image_topic 是 /image_raw
            # 这里的参数设置应该足够。但请注意 Gazebo 桥接的命名规范。
        ]
    )
    
    # -------------------------------------------------------------------------------------------------------------
    # Robot Spawner:
    # -------------------------------------------------------------------------------------------------------------

    xacro_file = os.path.join(morpheus_description_path, 'urdf', 'robot.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        remappings=[('/tf','tf_robot'),('/tf_static', 'tf_static_robot')]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc, 
                   '-x', '0.0', '-y', '4.0', '-z', '1.5',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
    )

    # ------------------------------------------------------------------------------------------------------------
    # Put your own ROS 2 Controlles here:
    # ------------------------------------------------------------------------------------------------------------
    
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
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_velocity_controller'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    load_suspension_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'suspension_controller'],
        output='screen'
    )

    load_robotic_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robotic_arm_controller'],
        output='screen'
    )

    # load_gripper_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'robotic_gripper_controller'],
    #     output='screen'
    # )

    activate_joint_state_controller = RegisterEventHandler(event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_controller],
            )
        )
    
    activate_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morpheus_nav2_path, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'  # Nav2 必须使用仿真时间
        }.items(),
    )
    
    activate_controllers = RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[#load_ackermann_controller,
                        load_forward_velocity_controller,
                        load_forward_position_controller,
                        load_suspension_controller,
                        load_robotic_arm_controller,
                        #load_gripper_controller,
                        ],
            )
        )
    
    # ------------------------------------------------------------------------------------------------------------
    # Gazebo Bridges:
    # ------------------------------------------------------------------------------------------------------------

    #Lidar Scan Bridge [/scan]
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    bridge_cloud = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    # Camera Bridge [/camera]
    bridge_camera_2i = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_2i@sensor_msgs/msg/Image@gz.msgs.Image'], 
        output='screen'
    )
    
    # 图像信息桥接 (需验证 Gazebo 输出的话题名)
    # 假设 Gazebo 发布的话题是 /camera_2i/camera_info
    bridge_camera_2i_info = Node( # <--- 修正 camera info bridge
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_2i/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'], 
        output='screen'
    )  

    bridge_camera_mini = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_mini@sensor_msgs/msg/Image@gz.msgs.Image'], 
        output='screen'
    ) 

    # Camera Info Bridge [/camera_info]
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'], 
        output='screen'
    ) 

    # Imu Bridge [/imu]
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'], 
        output='screen'
    ) 
    
    # ------------------------------------------------------------------------------------------------------------
    # Rviz 2:
    # ------------------------------------------------------------------------------------------------------------
    
    rviz_config_file = os.path.join(morpheus_simulation_path, 'launch', 'morpheus_config.rviz')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # ------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------
    
    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        activate_joint_state_controller,
        activate_controllers,
        controller,
        activate_nav2,
        bridge_camera_2i,  # 启用主摄像头
        #bridge_camera_mini, # 机械臂摄像头，非导航核心，可选启用
        #bridge_camera_info,
        bridge_scan,  # 启用 LIDAR 2D 扫描
        bridge_imu,  # 启用 IMU
        bridge_cloud, # 启用 LIDAR 点云
        aruco_node,
        #rviz, # 稍后可通过 Nav2 的配置启动
    ])
