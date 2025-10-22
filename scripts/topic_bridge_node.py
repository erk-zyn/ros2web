#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题桥接节点 - 实现ROS系统与Web端的双向通信

该节点通过WebSocket协议建立与Web前端的连接，提供以下功能：
1. Web端订阅/取消订阅ROS话题
2. 实时推送ROS话题消息到Web端
3. 接收Web端发布的ROS话题消息
4. 心跳检测和自动重连机制
5. 完善的异常处理和日志记录

作者: erk-zyn
邮箱: 1219534643@qq.com
项目主页: https://github.com/erk-zyn/ros2web
版本: 1.0.0
"""

import rospy
import asyncio
import websockets
import json
import threading
import time
import logging
from typing import Dict, Set, Any, Optional
from datetime import datetime
from std_msgs.msg import String, Int32, Float32, Bool
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('topic_bridge.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class MessageTypeRegistry:
    """消息类型注册表 - 管理ROS消息类型映射"""
    
    MESSAGE_TYPES = {
        'std_msgs/String': String,
        'std_msgs/Int32': Int32,
        'std_msgs/Float32': Float32,
        'std_msgs/Bool': Bool,
        'geometry_msgs/Twist': Twist,
        'geometry_msgs/Point': Point,
        'geometry_msgs/Quaternion': Quaternion,
        'sensor_msgs/LaserScan': LaserScan,
        'sensor_msgs/Image': Image,
        'nav_msgs/OccupancyGrid': OccupancyGrid,
        'nav_msgs/Odometry': Odometry,
        'nav_msgs/MapMetaData': MapMetaData,
    }
    
    @classmethod
    def get_message_class(cls, msg_type: str):
        """根据消息类型字符串获取对应的消息类"""
        return cls.MESSAGE_TYPES.get(msg_type)
    
    @classmethod
    def get_supported_types(cls) -> list:
        """获取支持的消息类型列表"""
        return list(cls.MESSAGE_TYPES.keys())


class ROSTopicManager:
    """ROS话题管理器 - 处理话题订阅和发布"""
    
    def __init__(self):
        self.subscribers: Dict[str, rospy.Subscriber] = {}
        self.publishers: Dict[str, rospy.Publisher] = {}
        self.topic_callbacks: Dict[str, callable] = {}
        
    def subscribe_topic(self, topic_name: str, msg_type: str, callback: callable) -> bool:
        """
        订阅ROS话题
        
        Args:
            topic_name: 话题名称
            msg_type: 消息类型
            callback: 回调函数
            
        Returns:
            bool: 订阅是否成功
        """
        try:
            if topic_name in self.subscribers:
                logger.warning(f"话题 {topic_name} 已经被订阅")
                return True
                
            msg_class = MessageTypeRegistry.get_message_class(msg_type)
            if not msg_class:
                logger.error(f"不支持的消息类型: {msg_type}")
                return False
                
            subscriber = rospy.Subscriber(topic_name, msg_class, callback)
            self.subscribers[topic_name] = subscriber
            self.topic_callbacks[topic_name] = callback
            
            logger.info(f"成功订阅话题: {topic_name} (类型: {msg_type})")
            return True
            
        except Exception as e:
            logger.error(f"订阅话题 {topic_name} 失败: {str(e)}")
            return False
    
    def unsubscribe_topic(self, topic_name: str) -> bool:
        """
        取消订阅ROS话题
        
        Args:
            topic_name: 话题名称
            
        Returns:
            bool: 取消订阅是否成功
        """
        try:
            if topic_name in self.subscribers:
                self.subscribers[topic_name].unregister()
                del self.subscribers[topic_name]
                del self.topic_callbacks[topic_name]
                logger.info(f"成功取消订阅话题: {topic_name}")
                return True
            else:
                logger.warning(f"话题 {topic_name} 未被订阅")
                return False
                
        except Exception as e:
            logger.error(f"取消订阅话题 {topic_name} 失败: {str(e)}")
            return False
    
    def create_publisher(self, topic_name: str, msg_type: str) -> bool:
        """
        创建ROS话题发布者
        
        Args:
            topic_name: 话题名称
            msg_type: 消息类型
            
        Returns:
            bool: 创建是否成功
        """
        try:
            if topic_name in self.publishers:
                logger.info(f"发布者 {topic_name} 已存在")
                return True
                
            msg_class = MessageTypeRegistry.get_message_class(msg_type)
            if not msg_class:
                logger.error(f"不支持的消息类型: {msg_type}")
                return False
                
            publisher = rospy.Publisher(topic_name, msg_class, queue_size=10)
            self.publishers[topic_name] = publisher
            
            logger.info(f"成功创建发布者: {topic_name} (类型: {msg_type})")
            return True
            
        except Exception as e:
            logger.error(f"创建发布者 {topic_name} 失败: {str(e)}")
            return False
    
    def publish_message(self, topic_name: str, message_data: dict) -> bool:
        """
        发布ROS消息
        
        Args:
            topic_name: 话题名称
            message_data: 消息数据
            
        Returns:
            bool: 发布是否成功
        """
        try:
            if topic_name not in self.publishers:
                logger.error(f"发布者 {topic_name} 不存在")
                return False
                
            publisher = self.publishers[topic_name]
            msg = self._create_message_from_data(publisher.data_class, message_data)
            
            if msg:
                publisher.publish(msg)
                logger.debug(f"成功发布消息到话题: {topic_name}")
                return True
            else:
                logger.error(f"创建消息失败: {topic_name}")
                return False
                
        except Exception as e:
            logger.error(f"发布消息到话题 {topic_name} 失败: {str(e)}")
            return False
    
    def _create_message_from_data(self, msg_class, data: dict):
        """根据数据字典创建ROS消息对象"""
        try:
            msg = msg_class()
            
            # 简单消息类型处理
            if hasattr(msg, 'data'):
                msg.data = data.get('data', '')
            
            # Twist消息处理
            elif msg_class == Twist:
                if 'linear' in data:
                    msg.linear.x = data['linear'].get('x', 0.0)
                    msg.linear.y = data['linear'].get('y', 0.0)
                    msg.linear.z = data['linear'].get('z', 0.0)
                if 'angular' in data:
                    msg.angular.x = data['angular'].get('x', 0.0)
                    msg.angular.y = data['angular'].get('y', 0.0)
                    msg.angular.z = data['angular'].get('z', 0.0)
            
            # Point消息处理
            elif msg_class == Point:
                msg.x = data.get('x', 0.0)
                msg.y = data.get('y', 0.0)
                msg.z = data.get('z', 0.0)
            
            # Odometry消息处理
            elif msg_class == Odometry:
                # 设置header
                if 'header' in data:
                    if 'frame_id' in data['header']:
                        msg.header.frame_id = data['header']['frame_id']
                    if 'stamp' in data['header']:
                        msg.header.stamp = rospy.Time.from_sec(data['header']['stamp'])
                
                # 设置pose
                if 'pose' in data and 'pose' in data['pose']:
                    pose_data = data['pose']['pose']
                    if 'position' in pose_data:
                        msg.pose.pose.position.x = pose_data['position'].get('x', 0.0)
                        msg.pose.pose.position.y = pose_data['position'].get('y', 0.0)
                        msg.pose.pose.position.z = pose_data['position'].get('z', 0.0)
                    if 'orientation' in pose_data:
                        msg.pose.pose.orientation.x = pose_data['orientation'].get('x', 0.0)
                        msg.pose.pose.orientation.y = pose_data['orientation'].get('y', 0.0)
                        msg.pose.pose.orientation.z = pose_data['orientation'].get('z', 0.0)
                        msg.pose.pose.orientation.w = pose_data['orientation'].get('w', 1.0)
                
                # 设置twist
                if 'twist' in data and 'twist' in data['twist']:
                    twist_data = data['twist']['twist']
                    if 'linear' in twist_data:
                        msg.twist.twist.linear.x = twist_data['linear'].get('x', 0.0)
                        msg.twist.twist.linear.y = twist_data['linear'].get('y', 0.0)
                        msg.twist.twist.linear.z = twist_data['linear'].get('z', 0.0)
                    if 'angular' in twist_data:
                        msg.twist.twist.angular.x = twist_data['angular'].get('x', 0.0)
                        msg.twist.twist.angular.y = twist_data['angular'].get('y', 0.0)
                        msg.twist.twist.angular.z = twist_data['angular'].get('z', 0.0)
            
            # MapMetaData消息处理
            elif msg_class == MapMetaData:
                msg.map_load_time = rospy.Time.from_sec(data.get('map_load_time', 0.0))
                msg.resolution = data.get('resolution', 0.05)
                msg.width = data.get('width', 0)
                msg.height = data.get('height', 0)
                
                if 'origin' in data:
                    origin_data = data['origin']
                    if 'position' in origin_data:
                        msg.origin.position.x = origin_data['position'].get('x', 0.0)
                        msg.origin.position.y = origin_data['position'].get('y', 0.0)
                        msg.origin.position.z = origin_data['position'].get('z', 0.0)
                    if 'orientation' in origin_data:
                        msg.origin.orientation.x = origin_data['orientation'].get('x', 0.0)
                        msg.origin.orientation.y = origin_data['orientation'].get('y', 0.0)
                        msg.origin.orientation.z = origin_data['orientation'].get('z', 0.0)
                        msg.origin.orientation.w = origin_data['orientation'].get('w', 1.0)
            
            # 其他复杂消息类型可以在这里扩展
            
            return msg
            
        except Exception as e:
            logger.error(f"创建消息对象失败: {str(e)}")
            return None
    
    def get_subscribed_topics(self) -> list:
        """获取已订阅的话题列表"""
        return list(self.subscribers.keys())
    
    def get_published_topics(self) -> list:
        """获取已创建发布者的话题列表"""
        return list(self.publishers.keys())


class WebSocketServer:
    """WebSocket服务器 - 处理与Web端的通信"""
    
    def __init__(self, host: str = "localhost", port: int = 8765):
        self.host = host
        self.port = port
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.ros_manager = ROSTopicManager()
        self.heartbeat_interval = 30  # 心跳间隔（秒）
        self.server = None
        
    async def register_client(self, websocket: websockets.WebSocketServerProtocol):
        """注册新的WebSocket客户端"""
        self.clients.add(websocket)
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"🔗 新客户端连接建立: {client_info} | 当前连接数: {len(self.clients)}")
        
        # 发送欢迎消息和支持的消息类型
        welcome_msg = {
            "type": "welcome",
            "message": "连接成功",
            "supported_message_types": MessageTypeRegistry.get_supported_types(),
            "timestamp": datetime.now().isoformat()
        }
        await self.send_to_client(websocket, welcome_msg)
        logger.info(f"📤 已向客户端 {client_info} 发送欢迎消息")
    
    async def unregister_client(self, websocket: websockets.WebSocketServerProtocol):
        """注销WebSocket客户端"""
        if websocket in self.clients:
            self.clients.remove(websocket)
            client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
            logger.info(f"🔌 客户端连接断开: {client_info} | 剩余连接数: {len(self.clients)}")
    
    async def send_to_client(self, websocket: websockets.WebSocketServerProtocol, message: dict):
        """向指定客户端发送消息"""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        msg_type = message.get("type", "unknown")
        
        try:
            await websocket.send(json.dumps(message, ensure_ascii=False))
            logger.info(f"📤 向客户端 {client_info} 发送消息: {msg_type}")
        except websockets.exceptions.ConnectionClosed:
            logger.warning(f"📤 向客户端 {client_info} 发送消息失败: 连接已关闭")
            await self.unregister_client(websocket)
        except Exception as e:
            logger.error(f"📤 向客户端 {client_info} 发送消息失败: {str(e)}")
    
    async def broadcast_message(self, message: dict):
        """向所有客户端广播消息"""
        if self.clients:
            msg_type = message.get("type", "unknown")
            logger.info(f"📢 向 {len(self.clients)} 个客户端广播消息: {msg_type}")
            
            disconnected_clients = set()
            for client in self.clients:
                try:
                    await client.send(json.dumps(message, ensure_ascii=False))
                except websockets.exceptions.ConnectionClosed:
                    disconnected_clients.add(client)
                except Exception as e:
                    client_info = f"{client.remote_address[0]}:{client.remote_address[1]}"
                    logger.error(f"📢 向客户端 {client_info} 广播消息失败: {str(e)}")
                    disconnected_clients.add(client)
            
            # 清理断开的连接
            for client in disconnected_clients:
                await self.unregister_client(client)
    
    def ros_message_callback(self, topic_name: str):
        """创建ROS消息回调函数"""
        def callback(msg):
            try:
                # 将ROS消息转换为字典格式
                message_dict = self._ros_msg_to_dict(msg)
                
                # 构造WebSocket消息
                ws_message = {
                    "type": "topic_message",
                    "topic": topic_name,
                    "message_type": type(msg).__name__,
                    "data": message_dict,
                    "timestamp": datetime.now().isoformat()
                }
                
                # 线程安全地调度异步任务
                self._schedule_broadcast(ws_message)
                
            except Exception as e:
                logger.error(f"处理ROS消息回调失败: {str(e)}")
        
        return callback
    
    def _schedule_broadcast(self, message: dict):
        """线程安全地调度广播消息任务"""
        try:
            # 获取当前事件循环
            loop = asyncio.get_event_loop()
            if loop.is_running():
                # 如果事件循环正在运行，使用call_soon_threadsafe
                loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(self.broadcast_message(message))
                )
            else:
                # 如果没有运行的事件循环，创建新的任务
                asyncio.create_task(self.broadcast_message(message))
        except RuntimeError:
            # 如果没有事件循环，尝试在新线程中处理
            try:
                import concurrent.futures
                with concurrent.futures.ThreadPoolExecutor() as executor:
                    future = executor.submit(self._sync_broadcast, message)
            except Exception as e:
                logger.error(f"调度广播消息失败: {str(e)}")
    
    def _sync_broadcast(self, message: dict):
        """同步方式广播消息（备用方案）"""
        try:
            # 创建新的事件循环来处理异步操作
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.broadcast_message(message))
            finally:
                loop.close()
        except Exception as e:
            logger.error(f"同步广播消息失败: {str(e)}")
    
    def _ros_msg_to_dict(self, msg) -> dict:
        """将ROS消息转换为字典格式"""
        try:
            result = {}
            
            # 处理简单数据类型
            if hasattr(msg, 'data'):
                result['data'] = msg.data
            
            # 处理Twist消息
            elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                result['linear'] = {
                    'x': msg.linear.x,
                    'y': msg.linear.y,
                    'z': msg.linear.z
                }
                result['angular'] = {
                    'x': msg.angular.x,
                    'y': msg.angular.y,
                    'z': msg.angular.z
                }
            
            # 处理Point消息
            elif hasattr(msg, 'x') and hasattr(msg, 'y') and hasattr(msg, 'z'):
                result['x'] = msg.x
                result['y'] = msg.y
                result['z'] = msg.z
            
            # 处理Odometry消息
            elif type(msg).__name__ == 'Odometry':
                result['header'] = {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp.to_sec()
                }
                result['child_frame_id'] = msg.child_frame_id
                result['pose'] = {
                    'pose': {
                        'position': {
                            'x': msg.pose.pose.position.x,
                            'y': msg.pose.pose.position.y,
                            'z': msg.pose.pose.position.z
                        },
                        'orientation': {
                            'x': msg.pose.pose.orientation.x,
                            'y': msg.pose.pose.orientation.y,
                            'z': msg.pose.pose.orientation.z,
                            'w': msg.pose.pose.orientation.w
                        }
                    },
                    'covariance': list(msg.pose.covariance)
                }
                result['twist'] = {
                    'twist': {
                        'linear': {
                            'x': msg.twist.twist.linear.x,
                            'y': msg.twist.twist.linear.y,
                            'z': msg.twist.twist.linear.z
                        },
                        'angular': {
                            'x': msg.twist.twist.angular.x,
                            'y': msg.twist.twist.angular.y,
                            'z': msg.twist.twist.angular.z
                        }
                    },
                    'covariance': list(msg.twist.covariance)
                }
            
            # 处理MapMetaData消息
            elif type(msg).__name__ == 'MapMetaData':
                result['map_load_time'] = msg.map_load_time.to_sec()
                result['resolution'] = msg.resolution
                result['width'] = msg.width
                result['height'] = msg.height
                result['origin'] = {
                    'position': {
                        'x': msg.origin.position.x,
                        'y': msg.origin.position.y,
                        'z': msg.origin.position.z
                    },
                    'orientation': {
                        'x': msg.origin.orientation.x,
                        'y': msg.origin.orientation.y,
                        'z': msg.origin.orientation.z,
                        'w': msg.origin.orientation.w
                    }
                }
            
            # 处理其他复杂消息类型
            else:
                # 通用处理：遍历消息的所有属性
                for slot in msg.__slots__:
                    attr_value = getattr(msg, slot)
                    if hasattr(attr_value, '__slots__'):
                        # 递归处理嵌套消息
                        result[slot] = self._ros_msg_to_dict(attr_value)
                    else:
                        result[slot] = attr_value
            
            return result
            
        except Exception as e:
            logger.error(f"转换ROS消息失败: {str(e)}")
            return {"error": "消息转换失败"}
    
    async def handle_client_message(self, websocket: websockets.WebSocketServerProtocol, message: str):
        """处理客户端消息"""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            logger.info(f"📥 收到客户端 {client_info} 消息: {msg_type}")
            
            if msg_type == "subscribe":
                await self.handle_subscribe(websocket, data)
            elif msg_type == "unsubscribe":
                await self.handle_unsubscribe(websocket, data)
            elif msg_type == "publish":
                await self.handle_publish(websocket, data)
            elif msg_type == "ping":
                await self.handle_ping(websocket, data)
            elif msg_type == "get_topics":
                await self.handle_get_topics(websocket, data)
            else:
                await self.send_error(websocket, f"未知的消息类型: {msg_type}")
                
        except json.JSONDecodeError:
            logger.warning(f"📥 收到客户端 {client_info} 无效JSON消息")
            await self.send_error(websocket, "无效的JSON格式")
        except Exception as e:
            logger.error(f"❌ 处理客户端 {client_info} 消息失败: {str(e)}")
            await self.send_error(websocket, f"处理消息失败: {str(e)}")
    
    async def handle_subscribe(self, websocket: websockets.WebSocketServerProtocol, data: dict):
        """处理话题订阅请求"""
        topic_name = data.get("topic")
        msg_type = data.get("message_type")
        
        if not topic_name or not msg_type:
            await self.send_error(websocket, "缺少必要参数: topic 或 message_type")
            return
        
        callback = self.ros_message_callback(topic_name)
        success = self.ros_manager.subscribe_topic(topic_name, msg_type, callback)
        
        response = {
            "type": "subscribe_response",
            "topic": topic_name,
            "message_type": msg_type,
            "success": success,
            "timestamp": datetime.now().isoformat()
        }
        
        await self.send_to_client(websocket, response)
    
    async def handle_unsubscribe(self, websocket: websockets.WebSocketServerProtocol, data: dict):
        """处理取消订阅请求"""
        topic_name = data.get("topic")
        
        if not topic_name:
            await self.send_error(websocket, "缺少必要参数: topic")
            return
        
        success = self.ros_manager.unsubscribe_topic(topic_name)
        
        response = {
            "type": "unsubscribe_response",
            "topic": topic_name,
            "success": success,
            "timestamp": datetime.now().isoformat()
        }
        
        await self.send_to_client(websocket, response)
    
    async def handle_publish(self, websocket: websockets.WebSocketServerProtocol, data: dict):
        """处理消息发布请求"""
        topic_name = data.get("topic")
        msg_type = data.get("message_type")
        message_data = data.get("data", {})
        
        if not topic_name or not msg_type:
            await self.send_error(websocket, "缺少必要参数: topic 或 message_type")
            return
        
        # 确保发布者存在
        if not self.ros_manager.create_publisher(topic_name, msg_type):
            await self.send_error(websocket, f"创建发布者失败: {topic_name}")
            return
        
        success = self.ros_manager.publish_message(topic_name, message_data)
        
        response = {
            "type": "publish_response",
            "topic": topic_name,
            "message_type": msg_type,
            "success": success,
            "timestamp": datetime.now().isoformat()
        }
        
        await self.send_to_client(websocket, response)
    
    async def handle_ping(self, websocket: websockets.WebSocketServerProtocol, data: dict):
        """处理心跳检测"""
        pong_response = {
            "type": "pong",
            "timestamp": datetime.now().isoformat()
        }
        await self.send_to_client(websocket, pong_response)
    
    async def handle_get_topics(self, websocket: websockets.WebSocketServerProtocol, data: dict):
        """处理获取话题列表请求"""
        response = {
            "type": "topics_list",
            "subscribed_topics": self.ros_manager.get_subscribed_topics(),
            "published_topics": self.ros_manager.get_published_topics(),
            "supported_message_types": MessageTypeRegistry.get_supported_types(),
            "timestamp": datetime.now().isoformat()
        }
        await self.send_to_client(websocket, response)
    
    async def send_error(self, websocket: websockets.WebSocketServerProtocol, error_message: str):
        """发送错误消息"""
        error_response = {
            "type": "error",
            "message": error_message,
            "timestamp": datetime.now().isoformat()
        }
        await self.send_to_client(websocket, error_response)
    
    async def client_handler(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """WebSocket客户端处理器"""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        await self.register_client(websocket)
        
        try:
            async for message in websocket:
                await self.handle_client_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"🔌 客户端 {client_info} 连接正常关闭")
        except Exception as e:
            logger.error(f"❌ 客户端 {client_info} 处理异常: {str(e)}")
        finally:
            await self.unregister_client(websocket)
    
    async def start_server(self):
        """启动WebSocket服务器"""
        try:
            self.server = await websockets.serve(
                self.client_handler,
                self.host,
                self.port,
                ping_interval=self.heartbeat_interval,
                ping_timeout=10
            )
            logger.info(f"WebSocket服务器启动成功: ws://{self.host}:{self.port}")
            
            # 启动心跳检测任务
            asyncio.create_task(self.heartbeat_task())
            
            # 保持服务器运行
            await self.server.wait_closed()
            
        except Exception as e:
            logger.error(f"启动WebSocket服务器失败: {str(e)}")
            raise
    
    async def heartbeat_task(self):
        """心跳检测任务"""
        while True:
            try:
                await asyncio.sleep(self.heartbeat_interval)
                
                if self.clients:
                    heartbeat_msg = {
                        "type": "heartbeat",
                        "timestamp": datetime.now().isoformat()
                    }
                    await self.broadcast_message(heartbeat_msg)
                    
            except Exception as e:
                logger.error(f"心跳检测任务异常: {str(e)}")
    
    async def stop_server(self):
        """停止WebSocket服务器"""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("WebSocket服务器已停止")


class TopicBridgeNode:
    """ROS话题桥接节点主类"""
    
    def __init__(self, node_name: str = "topic_bridge_node"):
        self.node_name = node_name
        self.ws_server = None
        self.server_thread = None
        
    def initialize_ros_node(self):
        """初始化ROS节点"""
        try:
            rospy.init_node(self.node_name, anonymous=True)
            logger.info(f"ROS节点初始化成功: {self.node_name}")
            
            # 注册关闭回调
            rospy.on_shutdown(self.shutdown_callback)
            
        except Exception as e:
            logger.error(f"ROS节点初始化失败: {str(e)}")
            raise
    
    def start_websocket_server(self, host: str = "localhost", port: int = 8765):
        """启动WebSocket服务器"""
        try:
            self.ws_server = WebSocketServer(host, port)
            
            # 在单独线程中运行异步服务器
            def run_server():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    loop.run_until_complete(self.ws_server.start_server())
                except Exception as e:
                    logger.error(f"WebSocket服务器运行异常: {str(e)}")
                finally:
                    loop.close()
            
            self.server_thread = threading.Thread(target=run_server, daemon=True)
            self.server_thread.start()
            
            logger.info("WebSocket服务器线程启动成功")
            
        except Exception as e:
            logger.error(f"启动WebSocket服务器失败: {str(e)}")
            raise
    
    def run(self):
        """运行节点主循环"""
        try:
            logger.info("话题桥接节点开始运行...")
            rospy.spin()
            
        except KeyboardInterrupt:
            logger.info("接收到中断信号，正在关闭节点...")
        except Exception as e:
            logger.error(f"节点运行异常: {str(e)}")
        finally:
            self.shutdown()
    
    def shutdown_callback(self):
        """ROS关闭回调"""
        logger.info("ROS节点正在关闭...")
        self.shutdown()
    
    def shutdown(self):
        """关闭节点和所有服务"""
        try:
            if self.ws_server:
                # 停止WebSocket服务器
                asyncio.run(self.ws_server.stop_server())
            
            logger.info("话题桥接节点已完全关闭")
            
        except Exception as e:
            logger.error(f"关闭节点时发生异常: {str(e)}")


def main():
    """主函数"""
    try:
        logger.info("=== ROS话题桥接节点启动 ===")
        logger.info("作者: erk-zyn")
        logger.info("项目主页: https://github.com/erk-zyn/ros2web")
        
        # 创建话题桥接节点
        bridge_node = TopicBridgeNode("topic_bridge_node")
        
        # 初始化ROS节点
        bridge_node.initialize_ros_node()
        
        # 启动WebSocket服务器
        bridge_node.start_websocket_server(host="0.0.0.0", port=8765)
        
        logger.info("=== 话题桥接节点启动成功 ===")
        
        # 运行节点
        bridge_node.run()
        
    except Exception as e:
        logger.error(f"节点启动失败: {str(e)}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())