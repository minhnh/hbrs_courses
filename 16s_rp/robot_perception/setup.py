#!/usr/bin/env python3

from distutils.core import setup

setup(
    name='robot_perception',
    version='1.0',
    description='Solution library for 16SS Robot Perception class',
    author='Minh Nguyen',
    author_email='minh.nguyen@smail.inf.h-brs.de',
    packages=['homography_estimation', 'fundamental_matrix_estimation'],
    package_dir={'' : 'src'},
)
