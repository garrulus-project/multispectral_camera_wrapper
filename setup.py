import os
from glob import glob
from setuptools import setup

package_name = "multispectral_camera_wrapper"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "sensor_msgs", "cv_bridge", "uas_msgs"],
    zip_safe=True,
    author="Mohammad Wasil",
    maintainer="Mohammad Wasil",
    maintainer_email="mohammad.wasil@h-brs.de",
    description="Multispectral camera wrapper package for ROS2",
    entry_points={
        "console_scripts": [
            "multispectral_camera_wrapper_node = multispectral_camera_wrapper.multispectral_camera_wrapper_node:main",
        ],
    },
)
