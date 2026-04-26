from setuptools import setup
import os
from glob import glob

package_name = "sterilink_localization"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pawan",
    maintainer_email="pawan@todo.todo",
    description="STERILINK localization stack",
    license="MIT",
    entry_points={
    'console_scripts': [
        'odom_icp          = sterilink_localization.odom_icp:main',
        'localiser_manager = sterilink_localization.localiser_manager:main',
        # keep your other nodes:
        'ekf_localization  = sterilink_localization.ekf_localization_node:main',
        'encoder_odometry  = sterilink_localization.encoder_odometry_node:main',
    ],
    },  
)
