from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocolumbus25_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikew',
    maintainer_email='mikew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robocolumbus25_wheel_controler_node = robocolumbus25_stuff.robocolumbus25_wheel_controler_node:main',
            'robocolumbus25_teleop_node = robocolumbus25_stuff.robocolumbus25_teleop_node:main',
            'robocolumbus25_imu_gps_node = robocolumbus25_stuff.robocolumbus25_imu_gps_node:main',
            'robocolumbus25_cone_node = robocolumbus25_stuff.robocolumbus25_cone_node:main',
            'robocolumbus25_nav_node = robocolumbus25_stuff.robocolumbus25_nav_node:main',
            'robocolumbus25_tof_node = robocolumbus25_stuff.robocolumbus25_tof_node:main',
        ],
    },
)
