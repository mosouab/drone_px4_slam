import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mo',
    maintainer_email='mo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hover = drone_control.offboard_test:main',
            'square = drone_control.square_test:main',
            'circle = drone_control.velocity_test:main',
            'body = drone_control.body_frame_test:main',
            'vision = drone_control.vision_aruco:main',
            'control = drone_control.control_land:main',
        ],
    },
)
