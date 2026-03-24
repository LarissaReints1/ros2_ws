from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')

    world = os.path.join(pkg_dir, 'worlds', 'empty.sdf')
    models_path = os.path.join(pkg_dir, 'models')
    
    # Set the model path for Ignition
    set_ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + models_path
    )

    # Launch Ignition Gazebo
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world],
        output='screen'
    )

    # Launch ROS2 bridge (with a short delay)
    ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                "config_file": os.path.join(pkg_dir, "config", "ros_gz_bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
    )
    ros2_bridge_delayed = TimerAction(period=2.0, actions=[ros2_bridge])

    # Joint state publisher GUI
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
    )

  

    return LaunchDescription([
        set_ign_model_path,
        ign_gazebo,
        ros2_bridge_delayed,
        joint_state_publisher,
        
    ])