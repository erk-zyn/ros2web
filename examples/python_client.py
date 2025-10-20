#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题桥接节点 - Python客户端示例

该示例展示如何使用Python客户端与ROS话题桥接节点进行通信，
包括订阅话题、发布消息、心跳检测等功能。

使用方法:
    python3 python_client.py

依赖:
    pip install websockets asyncio
"""

import asyncio
import websockets
import json
import logging
import signal
import sys
from datetime import datetime
from typing import Optional, Dict, Any

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ROSWebSocketClient:
    """ROS WebSocket客户端类"""
    
    def __init__(self, uri: str = "ws://localhost:8765"):
        self.uri = uri
        self.websocket: Optional[websockets.WebSocketServerProtocol] = None
        self.running = False
        self.reconnect_interval = 5  # 重连间隔（秒）
        self.heartbeat_interval = 30  # 心跳间隔（秒）
        
    async def connect(self):
        """连接到WebSocket服务器"""
        try:
            logger.info(f"正在连接到 {self.uri}")
            self.websocket = await websockets.connect(self.uri)
            logger.info("WebSocket连接已建立")
            return True
        except Exception as e:
            logger.error(f"连接失败: {e}")
            return False
    
    async def disconnect(self):
        """断开WebSocket连接"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None
            logger.info("WebSocket连接已断开")
    
    async def send_message(self, message: Dict[str, Any]):
        """发送消息到服务器"""
        if not self.websocket:
            logger.error("WebSocket未连接")
            return False
        
        try:
            message_json = json.dumps(message, ensure_ascii=False)
            await self.websocket.send(message_json)
            logger.debug(f"发送消息: {message}")
            return True
        except Exception as e:
            logger.error(f"发送消息失败: {e}")
            return False
    
    async def receive_messages(self):
        """接收服务器消息的协程"""
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    await self.handle_message(data)
                except json.JSONDecodeError as e:
                    logger.error(f"JSON解析失败: {e}")
                except Exception as e:
                    logger.error(f"处理消息失败: {e}")
        except websockets.exceptions.ConnectionClosed:
            logger.warning("WebSocket连接已关闭")
        except Exception as e:
            logger.error(f"接收消息异常: {e}")
    
    async def handle_message(self, message: Dict[str, Any]):
        """处理接收到的消息"""
        msg_type = message.get("type")
        timestamp = message.get("timestamp", "")
        
        if msg_type == "welcome":
            logger.info(f"收到欢迎消息: {message.get('message')}")
            supported_types = message.get('supported_message_types', [])
            logger.info(f"支持的消息类型: {', '.join(supported_types)}")
            
        elif msg_type == "topic_message":
            topic = message.get("topic")
            msg_type_name = message.get("message_type")
            data = message.get("data", {})
            logger.info(f"收到话题消息 [{topic}] ({msg_type_name})")
            logger.info(f"消息内容: {json.dumps(data, indent=2, ensure_ascii=False)}")
            
        elif msg_type == "subscribe_response":
            topic = message.get("topic")
            success = message.get("success")
            if success:
                logger.info(f"✅ 成功订阅话题: {topic}")
            else:
                logger.error(f"❌ 订阅话题失败: {topic}")
                
        elif msg_type == "unsubscribe_response":
            topic = message.get("topic")
            success = message.get("success")
            if success:
                logger.info(f"✅ 成功取消订阅话题: {topic}")
            else:
                logger.error(f"❌ 取消订阅话题失败: {topic}")
                
        elif msg_type == "publish_response":
            topic = message.get("topic")
            success = message.get("success")
            if success:
                logger.info(f"✅ 成功发布消息到话题: {topic}")
            else:
                logger.error(f"❌ 发布消息失败: {topic}")
                
        elif msg_type == "topics_list":
            subscribed = message.get("subscribed_topics", [])
            published = message.get("published_topics", [])
            logger.info(f"已订阅话题: {subscribed}")
            logger.info(f"已发布话题: {published}")
            
        elif msg_type == "heartbeat":
            logger.debug("收到服务器心跳")
            
        elif msg_type == "pong":
            logger.info("收到Pong响应")
            
        elif msg_type == "error":
            error_msg = message.get("message", "未知错误")
            logger.error(f"服务器错误: {error_msg}")
            
        else:
            logger.warning(f"未知消息类型: {msg_type}")
            logger.debug(f"消息内容: {json.dumps(message, indent=2, ensure_ascii=False)}")
    
    async def subscribe_topic(self, topic: str, message_type: str):
        """订阅ROS话题"""
        message = {
            "type": "subscribe",
            "topic": topic,
            "message_type": message_type
        }
        await self.send_message(message)
        logger.info(f"发送订阅请求: {topic} ({message_type})")
    
    async def unsubscribe_topic(self, topic: str):
        """取消订阅ROS话题"""
        message = {
            "type": "unsubscribe",
            "topic": topic
        }
        await self.send_message(message)
        logger.info(f"发送取消订阅请求: {topic}")
    
    async def publish_message(self, topic: str, message_type: str, data: Dict[str, Any]):
        """发布ROS消息"""
        message = {
            "type": "publish",
            "topic": topic,
            "message_type": message_type,
            "data": data
        }
        await self.send_message(message)
        logger.info(f"发送发布请求: {topic} ({message_type})")
    
    async def send_ping(self):
        """发送心跳检测"""
        message = {"type": "ping"}
        await self.send_message(message)
        logger.info("发送心跳检测")
    
    async def get_topics_list(self):
        """获取话题列表"""
        message = {"type": "get_topics"}
        await self.send_message(message)
        logger.info("请求话题列表")
    
    async def heartbeat_task(self):
        """心跳任务协程"""
        while self.running:
            try:
                await asyncio.sleep(self.heartbeat_interval)
                if self.websocket and not self.websocket.closed:
                    await self.send_ping()
            except Exception as e:
                logger.error(f"心跳任务异常: {e}")
    
    async def run_with_reconnect(self):
        """带重连功能的运行方法"""
        while self.running:
            try:
                if await self.connect():
                    # 启动消息接收和心跳任务
                    receive_task = asyncio.create_task(self.receive_messages())
                    heartbeat_task = asyncio.create_task(self.heartbeat_task())
                    
                    # 等待任务完成或异常
                    done, pending = await asyncio.wait(
                        [receive_task, heartbeat_task],
                        return_when=asyncio.FIRST_COMPLETED
                    )
                    
                    # 取消未完成的任务
                    for task in pending:
                        task.cancel()
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                
                # 如果仍在运行状态，等待后重连
                if self.running:
                    logger.info(f"将在 {self.reconnect_interval} 秒后尝试重连")
                    await asyncio.sleep(self.reconnect_interval)
                    
            except Exception as e:
                logger.error(f"运行异常: {e}")
                if self.running:
                    await asyncio.sleep(self.reconnect_interval)
    
    async def run(self):
        """运行客户端"""
        self.running = True
        
        try:
            # 启动带重连功能的运行任务
            main_task = asyncio.create_task(self.run_with_reconnect())
            
            # 启动示例任务
            demo_task = asyncio.create_task(self.demo_tasks())
            
            # 等待任务完成
            await asyncio.gather(main_task, demo_task)
            
        except KeyboardInterrupt:
            logger.info("收到中断信号，正在关闭客户端...")
        finally:
            await self.stop()
    
    async def stop(self):
        """停止客户端"""
        self.running = False
        await self.disconnect()
        logger.info("客户端已停止")
    
    async def demo_tasks(self):
        """示例任务 - 演示各种功能"""
        # 等待连接建立
        await asyncio.sleep(2)
        
        if not self.running:
            return
        
        try:
            # 1. 获取话题列表
            logger.info("=== 演示功能 1: 获取话题列表 ===")
            await self.get_topics_list()
            await asyncio.sleep(2)
            
            # 2. 订阅话题
            logger.info("=== 演示功能 2: 订阅话题 ===")
            await self.subscribe_topic("/cmd_vel", "geometry_msgs/Twist")
            await self.subscribe_topic("/chatter", "std_msgs/String")
            await asyncio.sleep(2)
            
            # 3. 发布消息
            logger.info("=== 演示功能 3: 发布消息 ===")
            
            # 发布Twist消息
            twist_data = {
                "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
            }
            await self.publish_message("/cmd_vel", "geometry_msgs/Twist", twist_data)
            await asyncio.sleep(1)
            
            # 发布String消息
            string_data = {"data": f"Hello from Python client! Time: {datetime.now().isoformat()}"}
            await self.publish_message("/chatter", "std_msgs/String", string_data)
            await asyncio.sleep(1)
            
            # 发布Point消息
            point_data = {"x": 1.0, "y": 2.0, "z": 3.0}
            await self.publish_message("/target_point", "geometry_msgs/Point", point_data)
            await asyncio.sleep(2)
            
            # 4. 连续发布消息（模拟控制循环）
            logger.info("=== 演示功能 4: 连续发布消息 ===")
            for i in range(5):
                if not self.running:
                    break
                    
                # 发布变化的速度命令
                twist_data = {
                    "linear": {"x": 0.5 + i * 0.1, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.2 * (-1) ** i}
                }
                await self.publish_message("/cmd_vel", "geometry_msgs/Twist", twist_data)
                await asyncio.sleep(3)
            
            # 5. 取消订阅
            logger.info("=== 演示功能 5: 取消订阅 ===")
            await self.unsubscribe_topic("/chatter")
            await asyncio.sleep(2)
            
            # 6. 再次获取话题列表
            logger.info("=== 演示功能 6: 再次获取话题列表 ===")
            await self.get_topics_list()
            
            # 7. 持续运行，接收消息
            logger.info("=== 持续运行，按Ctrl+C退出 ===")
            while self.running:
                await asyncio.sleep(10)
                # 定期发送心跳
                if self.websocket and not self.websocket.closed:
                    await self.send_ping()
                
        except Exception as e:
            logger.error(f"演示任务异常: {e}")


def signal_handler(signum, frame):
    """信号处理器"""
    logger.info("收到退出信号")
    sys.exit(0)


async def main():
    """主函数"""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 创建客户端
    client = ROSWebSocketClient("ws://localhost:8765")
    
    try:
        # 运行客户端
        await client.run()
    except KeyboardInterrupt:
        logger.info("程序被用户中断")
    except Exception as e:
        logger.error(f"程序异常: {e}")
    finally:
        await client.stop()


if __name__ == "__main__":
    print("🤖 ROS话题桥接节点 - Python客户端示例")
    print("=" * 50)
    print("该示例将演示以下功能:")
    print("1. 连接到WebSocket服务器")
    print("2. 订阅和取消订阅ROS话题")
    print("3. 发布ROS消息")
    print("4. 接收实时话题消息")
    print("5. 心跳检测和自动重连")
    print("=" * 50)
    print("请确保ROS话题桥接节点正在运行:")
    print("  rosrun topic_bridge topic_bridge_node.py")
    print("=" * 50)
    print("按Ctrl+C退出程序")
    print()
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序已退出")
    except Exception as e:
        print(f"\n程序异常退出: {e}")