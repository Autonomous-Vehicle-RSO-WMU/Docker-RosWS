from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolop_lane_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'msg'),
            glob('msg/*.msg')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nick',
    maintainer_email='nick@todo.todo',
    description='YOLOP lane detection ROS 2 package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolop_lane_node = yolop_lane_ros2.yolop_lane_node:main',
            'video_pub = yolop_lane_ros2.video_pub:main',
        ],
    },
)
