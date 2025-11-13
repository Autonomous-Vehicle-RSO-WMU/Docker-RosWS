from setuptools import setup

package_name = 'av_sim_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/spawn_and_perception.launch.py',
            'launch/nav2.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/rviz.rviz',
            'config/nav2_params.yaml'
        ]),
        ('share/' + package_name + '/data', ['data/waypoints.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Core ROS2 nodes for CARLA AV simulation (spawn, sensor fusion, YOLO detection, pure pursuit control).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_actors = av_sim_core.spawn_actors:main',
            'sensor_fusion = av_sim_core.sensor_fusion:main',
            'vision_detector = av_sim_core.vision_detector:main',
            'pure_pursuit = av_sim_core.pure_pursuit:main',
            'stop_controller = av_sim_core.stop_controller:main',
        ],
    },
)
