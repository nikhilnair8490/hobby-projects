from setuptools import setup

package_name = 'ultrasonic_sensor_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikhil',
    maintainer_email='nikhil@todo.todo',
    description='ROS2 interface package for HC-SR04 ultrasonic sensor',
    license='NA',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_sensor = ultrasonic_sensor_interface.ultrasonic_sensor:main'
        ],
    },
)
