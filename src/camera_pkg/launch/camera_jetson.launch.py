from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_params_jetson.yaml'
    )

    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='cam2image_node',
            name='cam2image_node',
            parameters=[config_file]
        )
    ])
