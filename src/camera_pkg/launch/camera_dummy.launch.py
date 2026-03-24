from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Launch : ros2 launch camera_pkg camera_dummy.launch.py


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_params_dummy.yaml'
    )

    # Node to capture and publish the image
    cam_node = Node(
        package='camera_pkg',
        executable='cam2image_node',
        name='cam2image_node',
        parameters=[config_file]
    )

    # Node to visualize the stream
    # This opens a window automatically
    viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=['/camera/image'] # Automatically subscribe to your topic
    )

    return LaunchDescription([
        cam_node,
        viewer_node #(Commented out to avoid opening window automatically)
    ])