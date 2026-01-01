from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'box_bot_description'
#Test the gitignore file to make sure it's working.
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        # Include world files (if you have any)
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='ROS 2 package for a simple box robot simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you write a python control node, add it here:
            # 'control_node = box_bot_description.control_node:main'
            'obstacle_avoider = box_bot_description.obstacleavoider:main',
        ],
    },
)
