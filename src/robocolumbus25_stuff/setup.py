from setuptools import find_packages, setup

package_name = 'robocolumbus25_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'robocolumbus25_wheel_controler_node = robocolumbus25_stuff.robocolumbus25_wheel_controler_node:main'
        ],
    },
)
