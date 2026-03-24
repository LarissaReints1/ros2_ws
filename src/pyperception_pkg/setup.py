from setuptools import setup
import os

package_name = 'pyperception_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='L. Reints',
    maintainer_email='l.reints@student.utwente.nl',
    description='ROS2 package for red target detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'red_target_depth_node = pyperception_pkg.red_target_depth_node:main',
            'red_target_node = pyperception_pkg.red_target_node:main',
        ],
    },
    data_files=[
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')]),
    ],
)