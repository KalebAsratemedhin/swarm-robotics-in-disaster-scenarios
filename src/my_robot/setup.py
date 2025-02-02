from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),

    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='kaleb',
    maintainer_email='risekab@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_controller = my_robot.swarm_controller:main',
            'controller = my_robot.controller:main',
            'obstacle_avoidance = my_robot.obstacle_avoidance:main',
            'circular_formation = my_robot.circular_formation:main',
            'leader_follower = my_robot.leader_follower:main',
            'position_publisher = my_robot.position_publisher:main',
            'collision_avoidance_flock = my_robot.collision_avoidance_flock:main'

        ],
    },
)
