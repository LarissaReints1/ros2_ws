import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Launch : ros2 launch perception_pkg perception.launch.py

def generate_launch_description():
    # Path to Perception Config
    perception_config = os.path.join(
        get_package_share_directory('perception_pkg'),
        'config',
        'perception_params.yaml'
    )

    # Path to Camera Config 
    camera_config = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_params_dummy.yaml'
    )

        # Static TF: map -> base_link
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=[
            '0', '0', '0',    # x y z
            '0', '0', '0',    # roll pitch yaw
            'map',            # parent frame
            'base_link'       # child frame
        ],
        output='screen'
    )


    # Static TF: base_link -> camera_link
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0.0', '0.4', '0.0',   # x y z
            '0.0', '0.0', '0.0',   # roll pitch yaw
            'base_link',          # parent frame
            'camera_link'         # child frame
        ],
        output='screen'
    )

    # Define the Perception Node
    perception_node = Node(
        package='perception_pkg',
        executable='perception_node',
        name='perception_node',
        parameters=[perception_config],
        output='screen'
    )

    # Define the Camera Node
    camera_node = Node(
        package='camera_pkg',
        executable='cam2image_node',
        name='cam2image_node',
        parameters=[camera_config],
        output='screen'
    )

    viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=['/perception/image']
    )

    return LaunchDescription([
        static_tf_map,
        static_tf_camera,
        camera_node,        # Comment out if no camera stream is needed
        perception_node,
        viewer_node
    ])
