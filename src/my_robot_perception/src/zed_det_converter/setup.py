from setuptools import find_packages, setup

package_name = 'zed_det_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Converts zed_msgs/ObjectsStamped to vision_msgs/Detection3DArray',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'converter_node = zed_det_converter.converter_node:main',
        ],
    },
)
