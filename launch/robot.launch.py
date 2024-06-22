import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = '/home/gursel/ros2_ws/src/diff_robot/urdf/diff_robot.urdf'
    rviz_config_file_path = '/home/gursel/ros2_ws/src/diff_robot/rviz/urdf_config.rviz'

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file_path,
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_file_path,
            description='Absolute path to rviz config file'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-file', LaunchConfiguration('model'), '-entity', 'diff_robot'],
             output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}]),

        Node(package='rviz2', executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')])
    ])
