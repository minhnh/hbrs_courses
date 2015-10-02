#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ball_detector_ros'],
    package_dir={'ball_detector_ros': 'ros/src'}
)

setup(**d)
