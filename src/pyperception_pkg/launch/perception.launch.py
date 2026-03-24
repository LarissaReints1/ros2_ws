# launch/full_system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include RViz launch from robot_description
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'rviz.launch.py'
            )
        )
    )

    # Include camera launch from camera_pkg
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('camera_pkg'),
                'launch',
                'camera_jetson.launch.py'
            )
        )
    )

    # Perception node
    red_target_node = Node(
        package='pyperception_pkg',
        executable='red_target_node',
        name='red_target_node',
        output='screen'
    )

    return LaunchDescription([
        rviz_launch,
        camera_launch,
        red_target_node
    ])