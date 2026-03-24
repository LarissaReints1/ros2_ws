from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction
from launch_ros.actions import Node
import os
import shutil

## ros2 launch camera_pkg calibration.launch.py

# ADJUST parameters:
# # ================ 
CAMERA_NAME = 'rx0_camera'
WIDTH = 1280
HEIGHT = 720
FPS = 25
CALLIBRATION_CORNERS = '10x7' 
CALLIBRATION_SCALE = '0.015'
# =========================

def rename_calibration_file(context, *args, **kwargs):
    tmp_tar = "/tmp/calibrationdata.tar.gz"
    if os.path.exists(tmp_tar):
        dest_file = f"config/info_{CAMERA_NAME}_{WIDTH}_{HEIGHT}.yaml"
        # Extract the calibration yaml
        import tarfile
        with tarfile.open(tmp_tar, "r:gz") as tar:
            for member in tar.getmembers():
                if member.name.endswith(".yaml"):
                    tar.extract(member, "config/")
                    extracted_path = os.path.join("config", member.name)
                    shutil.move(extracted_path, dest_file)
                    print(f"Calibration saved to {dest_file}")
                    break
    else:
        print(f"No calibration file found at {tmp_tar}")

def generate_launch_description():

    camera_node = Node(
        package='camera_pkg',
        executable='cam2image_node',
        name='camera_node',
        parameters=[{
            'stream_url': '0',
            'width': WIDTH,
            'height': HEIGHT,
            'fps': FPS,
            'camera_name': CAMERA_NAME
        }],
        output='screen'
    )

    calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='calibrator',
        arguments=[
            '--size', CALLIBRATION_CORNERS,
            '--square', CALLIBRATION_SCALE
        ],
        remappings=[
            ('image', '/camera/image'),
            ('camera', '/camera')
        ],
        output='screen'
    )

    # Delay rename 
    rename_action = TimerAction(
        period=1.0,  # seconds after launch
        actions=[OpaqueFunction(function=rename_calibration_file)]
    )

    return LaunchDescription([
        camera_node,
        calibration_node,
        rename_action
    ])