from setuptools import find_packages, setup
import os
import glob

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'models'), 
         glob(os.path.join('models', 'yolov8n.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cae-user',
    maintainer_email='cae-user@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
