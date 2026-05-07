from setuptools import setup

package_name = 'camera_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/camera_perception.yaml',
            'config/camera_perception_rear.yaml',
        ]),
        ('share/' + package_name + '/launch', ['launch/camera_perception.launch.py']),
        ('share/' + package_name + '/models', [
            'models/best.engine',
            'models/road_hazards_labels.yaml',
            'models/yolov8_road_hazards.onnx',
            'models/yolov8_road_hazards.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_MAINTAINER',
    maintainer_email='todo@example.com',
    description='Camera perception node (2D detections from images), with configurable placeholders for detector outputs.',
    license='TODO_LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_perception_node = camera_perception.camera_perception_node:main',
            'zed_od_adapter_node = camera_perception.zed_od_adapter_node:main',
        ],
    },
)
