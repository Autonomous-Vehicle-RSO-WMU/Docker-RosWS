from setuptools import setup
import os
from glob import glob

package_name = 'camera_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/192.168.1-metadata.json')),
        (os.path.join('share', package_name, 'models'), glob('models/best.engine')),
        (os.path.join('share', package_name, 'models'), glob('models/road_hazards_labels.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/yolov8_road_hazards.onnx')),
        (os.path.join('share', package_name, 'models'), glob('models/yolov8_road_hazards.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_MAINTAINER',
    maintainer_email='todo@example.com',
    description='Camera perception node (2D detections from images), extracted from camera_gated_clustering.',
    license='TODO_LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_perception_node = camera_perception.camera_perception_node:main',
            'zed_od_adapter_node = camera_perception.zed_od_adapter_node:main',
        ],
    },
)
