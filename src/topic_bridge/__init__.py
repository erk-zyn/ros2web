# -*- coding: utf-8 -*-
"""
ROS话题桥接节点核心模块

该模块提供了ROS话题桥接的核心功能，包括消息类型注册、话题管理、
WebSocket服务器和桥接节点等组件。

作者: erk-zyn
邮箱: 1219534643@qq.com
项目主页: https://github.com/erk-zyn/ros2web
"""

# 导入核心模块（这些模块在主节点文件中定义）
# 由于采用单文件架构，这里主要用于包的初始化

__version__ = "1.0.0"
__author__ = "erk-zyn"
__email__ = "1219534643@qq.com"

# 定义包的导出内容
__all__ = [
    'MessageTypeRegistry',
    'ROSTopicManager', 
    'WebSocketServer',
    'TopicBridgeNode'
]