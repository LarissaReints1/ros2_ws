from setuptools import setup
import os
from glob import glob

package_name = 'targeting_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 node to compute servo angles for targeting',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'servo_targeting_node = targeting_pkg.targeting_node:main',
        ],
    },
    data_files=[
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # Optional: models or other resources
        (os.path.join('share', package_name, 'models'), glob('models/agridrone/*')),
    ],
)