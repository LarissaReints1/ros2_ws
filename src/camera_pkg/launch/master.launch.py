from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Config files
    camera_config = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_params_dummy.yaml'
    )

    perception_config = os.path.join(
        get_package_share_directory('perception_pkg'),
        'config',
        'perception_params.yaml'
    )

    targeting_config = os.path.join(
        get_package_share_directory('targeting_pkg'),
        'config',
        'control_params.yaml'
    )

    return LaunchDescription([

        # --- Camera Node ---
        Node(
            package='camera_pkg',
            executable='cam2image_node',
            name='camera_node',
            parameters=[camera_config]
        ),

        # --- Perception Node ---
        Node(
            package='perception_pkg',
            executable='perception_node',
            name='perception_node',
            parameters=[perception_config]
        ),


        # --- Targeting Node ---
        Node(
            package='targeting_pkg',
            executable='targeting_node',
            name='targeting_node',
            parameters=[targeting_config]
        ),

        Node(
            package='targeting_pkg',
            executable='spraying_logic_node',
            name='spraying_logic_node',
            parameters=[targeting_config]
        ),

        # --- Actuation Nodes ---
        Node(
            package='actuation_pkg',
            executable='servo_driver_node',
            name='servo_driver_node',
        ),

        Node(
            package='actuation_pkg',
            executable='spray_driver_node',
            name='spray_driver_node'
        )

    ])
