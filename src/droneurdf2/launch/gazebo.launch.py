from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('droneurdf2')

    # Add your ROS package models folder to Ignition resource path
    ign_path = os.path.join(pkg_path, 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + ign_path

    # Path to Xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'droneurdf2.xacro')

    # Expand Xacro at runtime and treat as string for robot_state_publisher
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Start Gazebo Fortress
        ExecuteProcess(
            cmd=['ign', 'gazebo'],
            output='screen'
        ),

        # Publish robot description on /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawn the robot in Ignition Fortress
        # Spawn the robot 3 seconds after Ignition starts
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_drone',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-name', 'drone',
                        '-allow_renaming', 'true'
                    ],
                )
            ]
        )
    ])
