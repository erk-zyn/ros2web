#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS话题桥接节点安装配置
作者: erk-zyn
邮箱: 1219534643@qq.com
项目主页: https://github.com/erk-zyn/ros2web
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 获取包信息
d = generate_distutils_setup(
    packages=['topic_bridge'],
    package_dir={'': 'src'},
    requires=['rospy', 'websockets', 'asyncio', 'json-logging', 'pyyaml'],
    author='erk-zyn',
    author_email='1219534643@qq.com',
    url='https://github.com/erk-zyn/ros2web',
    description='ROS话题桥接节点，实现ROS系统与Web端的双向通信功能'
)

setup(**d)