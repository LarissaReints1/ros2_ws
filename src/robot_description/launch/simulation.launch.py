from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_description')

    # Paths
    xacro_file = os.path.join(pkg_dir, 'models', 'agridrone', 'agridrone.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'agridrone.rviz')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.sdf')
    models_path = os.path.join(pkg_dir, 'models')

    # Robot description
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Set model path for Ignition
    set_ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + models_path
    )

    # Launch Gazebo
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    # ROS-Gazebo bridge (delayed to wait for Gazebo)
    ros2_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                output='screen',
                parameters=[
                    {
                        "config_file": os.path.join(pkg_dir, "config", "ros_gz_bridge.yaml"),
                        "qos_overrides./tf_static.publisher.durability": "transient_local",
                    }
                ],
            )
        ]
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Optional: Joint state publisher GUI (only for testing in RViz2)
    joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
    )


    ## Perception node 
    # Paths to perception and camera configs
    perception_config = os.path.join(
        get_package_share_directory('perception_pkg'),
        'config',
        'perception_simulation.yaml'
    )
    
       # Perception Node
    perception_node = Node(
        package='perception_pkg',
        executable='perception_node',
        name='perception_node',
        parameters=[perception_config],
        output='screen'
    )

    viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=['/perception/image']
    )



    ## Targeting node

    target_config = os.path.join(
        get_package_share_directory('targeting_pkg'),
        'config',
        'control_params.yaml'
    )

    target_node = Node(
        package='targeting_pkg',
        executable='targeting_node',
        name='targeting_node',
        parameters=[target_config],
        output='screen',
        remappings=[
            ('/pan_tilt/joint_states', '/joint_states')  # robot_state_publisher listens here
        ]
    )

    # ---------------- ROS2 Control Node ----------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory('robot_description'), 'config', 'pan_tilt_controller.yaml')
        ],
        output='screen'
    )

    # Spawn the controller after ros2_control_node is up
    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_tilt_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        set_ign_model_path,
        ign_gazebo,
        ros2_bridge,
        robot_state_pub,
        rviz,
        #joint_gui,
        perception_node,
        target_node,
        viewer_node,
        
    ])