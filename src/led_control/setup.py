from setuptools import setup

package_name = 'led_control'

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
    description='ROS2 node to control LED based on distance of object',
    license='NA',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_control_node = led_control.led_control_node:main'
        ],
    },
)
