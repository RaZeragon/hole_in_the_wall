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
    maintainer_email='kward3@stevens.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hitw_controller = hitw_algorithm.hitw_controller:main',
        ],
    },
)
