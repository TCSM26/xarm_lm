from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xarm_lm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roger',
    maintainer_email='joserogelioruiz@gmail.com',
    description='xArm6 gaze stabilization via Levenberg-Marquardt IK (MoveIt Servo)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gaze_stabilizer  = xarm_lm.gaze_stabilizer:main',
            'base_disturbance = xarm_lm.base_disturbance:main',
            'initial_pose     = xarm_lm.initial_pose:main',
        ],
    },
)
