from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

agridrone_xacro_file = os.path.join(
    os.path.expanduser('~'), 'ros2_ws', 'src', 'targeting_pkg', 'urdf', 'agridrone.xacro')

urdf_file = xacro.process_file(agridrone_xacro_file).toxml()


config_file = os.path.join(
    get_package_share_directory('camera_pkg'),
    'config',
    'camera_params_jetson.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        # Camera Node
        Node(
            package='camera_pkg',
            executable='cam2image_node',
            name='cam2image_node',
            parameters=[config_file]
        ),

        # Red Target + Depth node
        Node(
            package='pyperception_pkg',          # package containing RedTargetDepthNode
            executable='red_target_node',
            name='red_target_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Servo targeting node
        Node(
            package='targeting_pkg',
            executable='servo_targeting_node',
            name='servo_targeting_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Arduino servo node
        Node(
            package='actuation_pkg',
            executable='arduino_servo_node',
            name='arduino_servo_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),
    ])