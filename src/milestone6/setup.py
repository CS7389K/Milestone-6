import os
from glob import glob

from setuptools import setup

package_name = 'milestone6'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.api',
        f'{package_name}.teleop',
        f'{package_name}.util',
        f'{package_name}.yolo',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Scouten',
    maintainer_email='yzb2@txstate.edu',
    description='Integrates teleoperation, YOLO perception, and LLaMa-based language capabilities from previous milestones \
        into a unified system deployed across the remote PC, Turtlebot Jetson, and Secondary Jetson, enabling end-to-end \
        robot control, perception, and interaction.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"teleop_publisher = {package_name}.teleop.publisher:main",
            f"teleop_subscriber = {package_name}.teleop.subscriber:main",
            f"yolo_publisher = {package_name}.yolo.publisher:main",
            f"yolo_subscriber = {package_name}.yolo.subscriber:main",
            f"part1 = {package_name}.part1:main",
            f"part2 = {package_name}.part2:main",
        ],
    },
)
