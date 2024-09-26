#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['multispectral_camera_wrapper',
             ],
    package_dir={'multispectral_camera_wrapper': 'src/multispectral_camera_wrapper',
                }
)

setup(**d)

