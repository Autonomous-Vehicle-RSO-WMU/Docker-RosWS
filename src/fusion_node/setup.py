from setuptools import setup

package_name = 'fusion_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/fusion_node.yaml']),
        ('share/' + package_name + '/launch', ['launch/fusion_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_MAINTAINER',
    maintainer_email='todo@example.com',
    description='LiDAR-camera fusion node (association + gating + visualization).',
    license='TODO_LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = fusion_node.fusion_node:main',
        ],
    },
)
