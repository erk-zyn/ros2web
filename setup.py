#!/usr/bin/env python
# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 获取包信息
d = generate_distutils_setup(
    packages=['topic_bridge'],
    package_dir={'': 'src'},
    requires=['rospy', 'websockets', 'asyncio', 'json-logging', 'pyyaml']
)

setup(**d)