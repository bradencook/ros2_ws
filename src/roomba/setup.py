import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roomba'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='The roomba package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roomba_node = roomba.roomba_node:main',
            'roomba_teleop = roomba.roomba_teleop:main',
            'imu_node = roomba.imu_node:main',
            'action_executor = roomba.action_executor:main',
            'bump_obstacle_node = roomba.bump_obstacle_node:main',
            'reasoning_node = roomba.reasoning_node:main',
            'telegram_node = roomba.telegram_node:main',
        ],
    },
)
