from setuptools import setup
import os

package_name = 'pyperception_pkg'

# Prepare launch files
launch_files = []
if os.path.exists('launch'):
    launch_files = [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')]

# Prepare models
model_files = []
if os.path.exists('models'):
    model_files = [os.path.join('models', f) for f in os.listdir('models') if f.endswith('.onnx')]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Larissa Reints',
    maintainer_email='l.reints@student.utwente.nl',
    description='ROS2 package for red target detection, depth estimation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'red_target_depth_node = pyperception_pkg.red_target_depth_node:main',
            'red_target_node = pyperception_pkg.red_target_node:main',
            'yolo_node = pyperception_pkg.yolo_node:main',
            'yolo_node2 = pyperception_pkg.yolo_node2:main',
            'yolo_node3 = pyperception_pkg.yolo_node3:main',
            'depth_node = pyperception_pkg.depth_node:main',
        ],
    },
    data_files=[
        # package.xml must be included
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', launch_files),
        # Models (TRT engines) and .engine
        ('share/' + package_name + '/models', model_files),
    ],
)