import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_path = '/home/gursel/ros2_ws/src/diff_robot/urdf/diff_robot.urdf'
    rviz_config_file_path = '/home/gursel/ros2_ws/src/diff_robot/urdf/rviz.rviz'
    slam_params_file_path = '/home/gursel/ros2_ws/src/diff_robot/map/slam_params.yaml'
    world_file_path = '/home/gursel/ros2_ws/src/diff_robot/world/office_small.world'
    controller_config_file_path = '/home/gursel/ros2_ws/src/diff_robot/control/diff_drive_controller.yaml'
    map_file = '/home/gursel/ros2_ws/src/diff_robot/src/my_map.yaml'
    #nav2_params_file_path = '/home/gursel/ros2_ws/src/diff_robot/config/nav2_params.yaml'  # Path to your Nav2 parameters file

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
        DeclareLaunchArgument(
            name='world',
            default_value=world_file_path,
            description='Absolute path to world file'),
        DeclareLaunchArgument(
            name='x',
            default_value='-0.142601',
            description='Initial x position of the robot'),
        DeclareLaunchArgument(
            name='y',
            default_value='-2.065040',
            description='Initial y position of the robot'),
        DeclareLaunchArgument(
            name='z',
            default_value='0.150008',
            description='Initial z position of the robot'),
        DeclareLaunchArgument(
            name='R',
            default_value='-0.000005',
            description='Initial roll orientation of the robot'),
        DeclareLaunchArgument(
            name='P',
            default_value='0.000040',
            description='Initial pitch orientation of the robot'),
        DeclareLaunchArgument(
            name='Y',
            default_value='-0.062120',
            description='Initial yaw orientation of the robot'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
            output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=[
                 '-file', LaunchConfiguration('model'),
                 '-entity', 'diff_robot',
                 '-x', LaunchConfiguration('x'),
                 '-y', LaunchConfiguration('y'),
                 '-z', LaunchConfiguration('z'),
                 '-R', LaunchConfiguration('R'),
                 '-P', LaunchConfiguration('P'),
                 '-Y', LaunchConfiguration('Y')],
             output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}],
             remappings=[("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")]),

        Node(package='rviz2', executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', LaunchConfiguration('rvizconfig')]),


        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': robot_desc}, controller_config_file_path],
             output='screen'),


            
    ])
