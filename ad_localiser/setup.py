from setuptools import setup
import os
from glob import glob

package_name = 'ad_localiser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahsanyusob',
    maintainer_email='ahm6464m@hs-coburg.de',
    description='ROS nodes for localising AD modelcars',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_optitrack = ad_localiser.odom_optitrack:main',
            'localiser_manager = ad_localiser.localiser_manager:main',
        ],
    },
)
