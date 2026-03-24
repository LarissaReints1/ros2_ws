from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('droneurdf2')

    world_file = os.path.join(pkg_path, 'worlds', 'robot_world.sdf')
    robot_sdf_file = os.path.join(pkg_path, 'sdf', 'my_robot.sdf')

    # Optional: add package models folder to Ignition resource path
    ign_models_path = os.path.join(pkg_path, 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + ign_models_path

    return LaunchDescription([
        # Start Ignition Gazebo with your world
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '--verbose'],
            output='screen'
        ),

       
    ])