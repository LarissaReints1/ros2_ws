from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=[
                '--x', '0.1',
                '--y', '0',
                '--z', '0.2',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame'
            ]
        ),
    ])