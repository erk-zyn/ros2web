# ROS话题桥接节点 - WebSocket通信协议规范

## 概述

本文档详细描述了ROS话题桥接节点与Web前端之间的WebSocket通信协议。该协议支持双向通信，允许Web端订阅ROS话题、发布消息以及接收实时数据。

## 连接信息

- **协议**: WebSocket (ws://)
- **默认地址**: ws://localhost:8765
- **支持的数据格式**: JSON

## 消息格式规范

所有WebSocket消息都采用JSON格式，包含以下基本字段：

```json
{
  "type": "消息类型",
  "timestamp": "ISO格式时间戳",
  // 其他特定字段...
}
```

## 客户端发送的消息类型

### 1. 订阅话题 (subscribe)

订阅指定的ROS话题，开始接收该话题的消息。

**请求格式:**
```json
{
  "type": "subscribe",
  "topic": "/cmd_vel",
  "message_type": "geometry_msgs/Twist"
}
```

**字段说明:**
- `type`: 固定值 "subscribe"
- `topic`: ROS话题名称，必须以"/"开头
- `message_type`: ROS消息类型，格式为"包名/消息类型"

**响应格式:**
```json
{
  "type": "subscribe_response",
  "topic": "/cmd_vel",
  "message_type": "geometry_msgs/Twist",
  "success": true,
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 2. 取消订阅 (unsubscribe)

取消对指定ROS话题的订阅。

**请求格式:**
```json
{
  "type": "unsubscribe",
  "topic": "/cmd_vel"
}
```

**字段说明:**
- `type`: 固定值 "unsubscribe"
- `topic`: 要取消订阅的ROS话题名称

**响应格式:**
```json
{
  "type": "unsubscribe_response",
  "topic": "/cmd_vel",
  "success": true,
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 3. 发布消息 (publish)

向指定的ROS话题发布消息。

**请求格式:**
```json
{
  "type": "publish",
  "topic": "/cmd_vel",
  "message_type": "geometry_msgs/Twist",
  "data": {
    "linear": {
      "x": 1.0,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.5
    }
  }
}
```

**字段说明:**
- `type`: 固定值 "publish"
- `topic`: ROS话题名称
- `message_type`: ROS消息类型
- `data`: 消息数据，格式根据消息类型而定

**响应格式:**
```json
{
  "type": "publish_response",
  "topic": "/cmd_vel",
  "message_type": "geometry_msgs/Twist",
  "success": true,
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 4. 心跳检测 (ping)

发送心跳信号以维持连接活性。

**请求格式:**
```json
{
  "type": "ping"
}
```

**响应格式:**
```json
{
  "type": "pong",
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 5. 获取话题列表 (get_topics)

获取当前已订阅和已发布的话题列表。

**请求格式:**
```json
{
  "type": "get_topics"
}
```

**响应格式:**
```json
{
  "type": "topics_list",
  "subscribed_topics": ["/cmd_vel", "/scan"],
  "published_topics": ["/status"],
  "supported_message_types": [
    "std_msgs/String",
    "std_msgs/Int32",
    "geometry_msgs/Twist",
    "sensor_msgs/LaserScan"
  ],
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

## 服务器发送的消息类型

### 1. 欢迎消息 (welcome)

客户端连接成功后收到的欢迎消息。

```json
{
  "type": "welcome",
  "message": "连接成功",
  "supported_message_types": [
    "std_msgs/String",
    "std_msgs/Int32",
    "std_msgs/Float32",
    "std_msgs/Bool",
    "geometry_msgs/Twist",
    "geometry_msgs/Point",
    "geometry_msgs/Quaternion",
    "sensor_msgs/LaserScan",
    "sensor_msgs/Image",
    "nav_msgs/Odometry",
    "nav_msgs/MapMetaData",
    "nav_msgs/OccupancyGrid"
  ],
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 2. 话题消息 (topic_message)

从已订阅的ROS话题接收到的实时消息。

```json
{
  "type": "topic_message",
  "topic": "/cmd_vel",
  "message_type": "Twist",
  "data": {
    "linear": {
      "x": 1.0,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.5
    }
  },
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 3. 心跳消息 (heartbeat)

服务器定期发送的心跳消息。

```json
{
  "type": "heartbeat",
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

### 4. 错误消息 (error)

当发生错误时服务器发送的错误信息。

```json
{
  "type": "error",
  "message": "不支持的消息类型: unknown_msgs/Unknown",
  "timestamp": "2024-01-15T10:30:00.000Z"
}
```

## 支持的ROS消息类型

### 标准消息类型 (std_msgs)

#### String
```json
{
  "data": "Hello World"
}
```

#### Int32
```json
{
  "data": 42
}
```

#### Float32
```json
{
  "data": 3.14
}
```

#### Bool
```json
{
  "data": true
}
```

### 几何消息类型 (geometry_msgs)

#### Twist (速度控制)
```json
{
  "linear": {
    "x": 1.0,
    "y": 0.0,
    "z": 0.0
  },
  "angular": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.5
  }
}
```

#### Point (三维点)
```json
{
  "x": 1.0,
  "y": 2.0,
  "z": 3.0
}
```

#### Quaternion (四元数)
```json
{
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "w": 1.0
}
```

### 传感器消息类型 (sensor_msgs)

#### LaserScan (激光雷达数据)
```json
{
  "header": {
    "seq": 1,
    "stamp": {
      "secs": 1642248600,
      "nsecs": 0
    },
    "frame_id": "laser"
  },
  "angle_min": -3.14159,
  "angle_max": 3.14159,
  "angle_increment": 0.00872665,
  "time_increment": 0.0,
  "scan_time": 0.1,
  "range_min": 0.1,
  "range_max": 10.0,
  "ranges": [1.0, 1.1, 1.2, 1.3],
  "intensities": [100, 110, 120, 130]
}
```

### 导航消息类型 (nav_msgs)

#### Odometry (里程计数据)
```json
{
  "header": {
    "seq": 1,
    "stamp": {
      "secs": 1642248600,
      "nsecs": 0
    },
    "frame_id": "odom"
  },
  "child_frame_id": "base_link",
  "pose": {
    "pose": {
      "position": {
        "x": 1.0,
        "y": 2.0,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.707,
        "w": 0.707
      }
    },
    "covariance": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
  },
  "twist": {
    "twist": {
      "linear": {
        "x": 0.5,
        "y": 0.0,
        "z": 0.0
      },
      "angular": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.2
      }
    },
    "covariance": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
  }
}
```

#### MapMetaData (地图元数据)
```json
{
  "map_load_time": {
    "secs": 1642248600,
    "nsecs": 0
  },
  "resolution": 0.05,
  "width": 100,
  "height": 100,
  "origin": {
    "position": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  }
}
```

#### OccupancyGrid (占用栅格地图)
```json
{
  "header": {
    "seq": 1,
    "stamp": {
      "secs": 1642248600,
      "nsecs": 0
    },
    "frame_id": "map"
  },
  "info": {
    "map_load_time": {
      "secs": 1642248600,
      "nsecs": 0
    },
    "resolution": 0.05,
    "width": 100,
    "height": 100,
    "origin": {
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  },
  "data": [0, 0, 0, 100, -1]
}
```

## 连接管理

### 连接建立
1. 客户端连接到WebSocket服务器
2. 服务器发送欢迎消息，包含支持的消息类型列表
3. 客户端可以开始发送订阅、发布等请求

### 心跳机制
- 服务器每30秒发送一次心跳消息
- 客户端可以发送ping消息进行主动心跳检测
- 连接超时时间为10秒

### 自动重连
- 当连接断开时，客户端应实现自动重连机制
- 重连成功后需要重新订阅之前的话题
- 建议使用指数退避算法控制重连间隔

### 错误处理
- 所有错误都会通过error类型消息返回
- 常见错误类型：
  - 不支持的消息类型
  - 缺少必要参数
  - ROS话题操作失败
  - JSON格式错误

## 使用示例

### JavaScript客户端示例

```javascript
class ROSWebSocketClient {
  constructor(url = 'ws://localhost:8765') {
    this.url = url;
    this.ws = null;
    this.reconnectInterval = 1000;
    this.maxReconnectInterval = 30000;
    this.reconnectDecay = 1.5;
    this.timeoutInterval = 2000;
  }

  connect() {
    this.ws = new WebSocket(this.url);
    
    this.ws.onopen = (event) => {
      console.log('WebSocket连接已建立');
      this.reconnectInterval = 1000;
    };
    
    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data);
      this.handleMessage(message);
    };
    
    this.ws.onclose = (event) => {
      console.log('WebSocket连接已关闭');
      this.reconnect();
    };
    
    this.ws.onerror = (error) => {
      console.error('WebSocket错误:', error);
    };
  }

  reconnect() {
    setTimeout(() => {
      console.log('尝试重新连接...');
      this.connect();
      this.reconnectInterval = Math.min(
        this.reconnectInterval * this.reconnectDecay,
        this.maxReconnectInterval
      );
    }, this.reconnectInterval);
  }

  subscribe(topic, messageType) {
    const message = {
      type: 'subscribe',
      topic: topic,
      message_type: messageType
    };
    this.send(message);
  }

  publish(topic, messageType, data) {
    const message = {
      type: 'publish',
      topic: topic,
      message_type: messageType,
      data: data
    };
    this.send(message);
  }

  send(message) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  handleMessage(message) {
    switch (message.type) {
      case 'welcome':
        console.log('收到欢迎消息:', message);
        break;
      case 'topic_message':
        console.log('收到话题消息:', message);
        break;
      case 'heartbeat':
        console.log('收到心跳消息');
        break;
      case 'error':
        console.error('收到错误消息:', message.message);
        break;
      default:
        console.log('未知消息类型:', message);
    }
  }
}

// 使用示例
const client = new ROSWebSocketClient();
client.connect();

// 订阅速度话题
client.subscribe('/cmd_vel', 'geometry_msgs/Twist');

// 发布速度命令
client.publish('/cmd_vel', 'geometry_msgs/Twist', {
  linear: { x: 1.0, y: 0.0, z: 0.0 },
  angular: { x: 0.0, y: 0.0, z: 0.5 }
});
```

## 性能考虑

### 消息频率限制
- 建议控制高频话题的发布频率，避免网络拥塞
- 对于传感器数据，可以考虑降采样或压缩

### 并发连接
- 服务器支持多个客户端同时连接
- 每个客户端的订阅是独立的

### 内存管理
- 服务器会自动清理断开连接的客户端
- 长时间运行时注意监控内存使用情况

## 安全注意事项

1. **网络安全**: 在生产环境中考虑使用WSS (WebSocket Secure)
2. **访问控制**: 根据需要实现身份验证和授权机制
3. **输入验证**: 服务器会验证所有输入参数的有效性
4. **资源限制**: 考虑实现连接数和消息频率限制

## 故障排除

### 常见问题

1. **连接失败**
   - 检查服务器是否正在运行
   - 确认端口号和地址是否正确
   - 检查防火墙设置

2. **订阅失败**
   - 确认话题名称格式正确（以"/"开头）
   - 检查消息类型是否在支持列表中
   - 确认ROS节点是否正常运行

3. **消息接收不到**
   - 检查话题是否有发布者
   - 确认订阅是否成功
   - 查看服务器日志获取详细错误信息

### 调试工具

- 使用浏览器开发者工具查看WebSocket通信
- 检查服务器日志文件 `topic_bridge.log`
- 使用ROS命令行工具验证话题状态：
  ```bash
  rostopic list
  rostopic info /topic_name
  rostopic echo /topic_name
  ```

## 版本信息

- **协议版本**: 1.0.0
- **最后更新**: 2024-01-15
- **兼容的ROS版本**: ROS Melodic, ROS Noetic