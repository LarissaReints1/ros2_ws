from setuptools import setup

package_name = 'actuation_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'Jetson.GPIO'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 node to control 2 servos on Jetson Nano',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = actuation_pkg.servo_node:main',
            'angle_test_publisher = actuation_pkg.angle_test_publisher:main',
            'arduino_servo_node = actuation_pkg.arduino_servo_node:main',
        ],
    },
    data_files=[
    ('share/actuation_pkg/config', ['config/test_angles.yaml']),
],
)
