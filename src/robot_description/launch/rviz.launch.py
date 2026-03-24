from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')

    # Paths
    xacro_file = os.path.join(pkg_dir, 'models', 'agridrone', 'agridrone.xacro')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'agridrone.rviz') 

    # Robot description from Xacro
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Robot state publisher publishes TF from /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description
            }],
            remappings=[('/joint_states', '/target/joint_states')]
        ),

        # Joint state publisher GUI (optional sliders to control pan/tilt)
        #Node(
        #    package='joint_state_publisher_gui',
        #    executable='joint_state_publisher_gui',
        #    name='joint_state_publisher_gui',
        #    output='screen'
        #),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])