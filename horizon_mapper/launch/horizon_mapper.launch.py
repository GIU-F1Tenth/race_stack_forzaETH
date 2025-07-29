#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='false',
        description='Enable detailed logging for debugging'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='car_state/odom',
        description='Odometry topic name'
    )
    
    horizon_N_arg = DeclareLaunchArgument(
        'horizon_N',
        default_value='8',
        description='MPC prediction horizon length'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('horizon_mapper')
    
    # Define trajectory file paths
    optimal_trajectory_path = PathJoinSubstitution([
        pkg_share, 'data', 'optimal_trajectory.csv'
    ])
    
    reference_trajectory_path = PathJoinSubstitution([
        pkg_share, 'data', 'ref_trajectory.csv'
    ])
    
    # Create the horizon mapper node
    horizon_mapper_node = Node(
        package='horizon_mapper',
        executable='horizon_mapper_node',
        name='horizon_mapper_node',
        output='screen',
        parameters=[{
            'enable_logging': LaunchConfiguration('enable_logging'),
            'optimal_trajectory_path': optimal_trajectory_path,
            'reference_trajectory_path': reference_trajectory_path,
            'horizon_N': LaunchConfiguration('horizon_N'),
            'horizon_T': 0.5,
            'wheelbase': 0.33,
            'max_steering_angle': 0.5,
            'min_speed': 0.1,
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
        remappings=[
            # Remap topics if needed
        ]
    )
    
    return LaunchDescription([
        enable_logging_arg,
        odom_topic_arg,
        horizon_N_arg,
        horizon_mapper_node,
    ])
