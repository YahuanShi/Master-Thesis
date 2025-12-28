import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    full = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_pkg = get_package_share_directory('morpheus_moveit_config')
    desc_pkg = get_package_share_directory('morpheus_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description = Command([
        'xacro ', os.path.join(desc_pkg, 'urdf', 'robot.xacro')])

    srdf_file = os.path.join(moveit_pkg, 'config', 'morpheus.srdf')
    with open(srdf_file) as f:
        robot_description_semantic = f.read()

    kinematics_yaml = load_yaml('morpheus_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('morpheus_moveit_config', 'config/joint_limits.yaml')
    ompl_yaml = load_yaml('morpheus_moveit_config', 'config/ompl_planning.yaml')
    controllers_yaml = load_yaml('morpheus_moveit_config', 'config/moveit_controllers.yaml')

    moveit_config = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        'use_sim_time': use_sim_time,
    }

    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_yaml,
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config,
            planning_pipeline,
            trajectory_execution,
            planning_scene_monitor,
            controllers_yaml,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'),
        move_group_node,
    ])
