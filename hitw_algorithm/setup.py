import os
from glob import glob
from setuptools import setup

package_name = 'hitw_algorithm'
submodule_name = 'hitw_algorithm/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='razeragon',
    maintainer_email='kward2787@gmail.com',
    description='Image processing algorithm for determining if the robot can fit inside the hole.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotpose_service = hitw_algorithm.calculateRobotPose_srv:main',
            'robotpose_client = hitw_algorithm.calculateRobotPose_cli:main',
        ],
    },
)
