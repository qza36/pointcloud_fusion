# PointCloud Fusion

[English](#english) | [中文](#中文)

---

## English

### Overview

`pointcloud_fusion` is a ROS 2 package designed to synchronize and fuse point clouds from multiple LiDAR sensors. It uses TF2 transformations to convert point clouds to a common reference frame and merges them into a single unified point cloud.

### Features

- **Multi-LiDAR Synchronization**: Synchronizes point clouds from two LiDAR sensors using approximate time synchronization
- **TF2 Integration**: Automatically transforms point clouds to a target reference frame using TF2
- **Point Cloud Fusion**: Merges multiple point clouds into a single unified output
- **ROS 2 Native**: Built specifically for ROS 2 with modern C++ practices
- **Configurable**: Flexible parameter configuration for different sensor setups

### Requirements

#### System Requirements
- ROS 2 (tested with ROS 2 humble)
- Ubuntu 22.04 or later (or compatible Linux distribution)
- CMake 3.22 or later

#### Dependencies
- `rclcpp` - ROS 2 C++ client library
- `sensor_msgs` - ROS 2 sensor messages
- `message_filters` - Message synchronization
- `pcl_ros` - Point Cloud Library ROS interface
- `pcl_conversions` - PCL/ROS conversions
- `tf2_ros` - TF2 ROS bindings
- `Eigen3` - Linear algebra library

### Installation

1. **Create a ROS 2 workspace** (if you don't have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone the repository**:
   ```bash
   git clone https://github.com/qza36/pointcloud_fusion.git
   ```

3. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select pointcloud_fusion
   ```

5. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### Usage

#### Launch the Node

```bash
ros2 run pointcloud_fusion fusion_node
```

#### With Custom Parameters

```bash
ros2 run pointcloud_fusion fusion_node --ros-args \
  -p target_frame:=base_link \
  -p lidar_topic:=/BottomLidar/livox/lidar/pointcloud \
  -p rear_lidar_topic:=/TopLidar/livox/lidar/pointcloud \
  -p lidar_frame:=livox_frame_bottom \
  -p rear_lidar_frame:=livox_frame_top
```

### Configuration

The node supports the following ROS 2 parameters:

| Parameter | Type | Default Value | Description |
|-----------|------|---------------|-------------|
| `target_frame` | string | `base_link` | Target reference frame for point cloud fusion |
| `lidar_topic` | string | `/BottomLidar/livox/lidar/pointcloud` | First LiDAR point cloud topic |
| `rear_lidar_topic` | string | `/TopLidar/livox/lidar/pointcloud` | Second LiDAR point cloud topic |
| `lidar_frame` | string | `livox_frame_bottom` | Frame ID of the first LiDAR |
| `rear_lidar_frame` | string | `livox_frame_top` | Frame ID of the second LiDAR |

### Topics

#### Subscribed Topics
- First LiDAR point cloud (default: `/BottomLidar/livox/lidar/pointcloud`)
  - Type: `sensor_msgs/msg/PointCloud2`
  - Point cloud from the first LiDAR sensor

- Second LiDAR point cloud (default: `/TopLidar/livox/lidar/pointcloud`)
  - Type: `sensor_msgs/msg/PointCloud2`
  - Point cloud from the second LiDAR sensor

#### Published Topics
- `/points/fused`
  - Type: `sensor_msgs/msg/PointCloud2`
  - Fused point cloud in the target reference frame

### How It Works

1. **Subscription**: The node subscribes to two point cloud topics from different LiDAR sensors
2. **Synchronization**: Uses approximate time synchronization to match point clouds from both sensors
3. **Transformation**: Transforms both point clouds to the target reference frame using TF2
4. **Fusion**: Concatenates the transformed point clouds into a single unified point cloud
5. **Publication**: Publishes the fused point cloud to the output topic

### Architecture

```
┌─────────────────┐
│  LiDAR Sensor 1 │
│  (Bottom)       │
└────────┬────────┘
         │ PointCloud2
         ▼
    ┌────────────────────┐
    │                    │
    │  Message Filters   │◄─────┐
    │  (Synchronizer)    │      │ PointCloud2
    │                    │      │
    └────────┬───────────┘      │
             │                  │
             │            ┌─────┴────────┐
             │            │ LiDAR Sensor 2│
             │            │ (Top)         │
             ▼            └───────────────┘
    ┌─────────────────────┐
    │  TF2 Transform      │
    │  to target_frame    │
    └────────┬────────────┘
             │
             ▼
    ┌─────────────────────┐
    │  Point Cloud Fusion │
    └────────┬────────────┘
             │
             ▼
    ┌─────────────────────┐
    │  /points/fused      │
    │  (Output)           │
    └─────────────────────┘
```

### Development

#### Building for Development
```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_fusion --symlink-install
```

#### Running Tests
```bash
cd ~/ros2_ws
colcon test --packages-select pointcloud_fusion
```

### Troubleshooting

#### Transform Errors
If you encounter transform errors:
- Ensure TF2 transformations are properly broadcasted between sensor frames and the target frame
- Check that frame IDs match between the point cloud messages and TF tree
- Use `ros2 run tf2_ros tf2_echo <source_frame> <target_frame>` to verify transforms

#### Synchronization Issues
If point clouds are not being synchronized:
- Verify both LiDAR topics are publishing data
- Check that timestamps on both topics are reasonably close (within ~100ms)
- Adjust the synchronization queue size if needed

### License

TODO: License declaration

### Maintainer

- **Maintainer**: ZhiangQi
- **Email**: an9qi@yandex.com

### Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

---

## 中文

### 概述

`pointcloud_fusion` 是一个 ROS 2 软件包，用于同步和融合来自多个激光雷达传感器的点云数据。它使用 TF2 变换将点云转换到共同的参考坐标系，并将它们合并为单个统一的点云。

### 功能特性

- **多激光雷达同步**: 使用近似时间同步算法同步来自两个激光雷达传感器的点云
- **TF2 集成**: 使用 TF2 自动将点云转换到目标参考坐标系
- **点云融合**: 将多个点云合并为单个统一输出
- **原生 ROS 2**: 专为 ROS 2 构建，采用现代 C++ 实践
- **可配置**: 灵活的参数配置，适用于不同的传感器设置

### 系统要求

#### 系统环境
- ROS 2（已在 ROS 2 humble上测试）
- Ubuntu 22.04 或更高版本（或兼容的 Linux 发行版）
- CMake 3.22 或更高版本

#### 依赖项
- `rclcpp` - ROS 2 C++ 客户端库
- `sensor_msgs` - ROS 2 传感器消息
- `message_filters` - 消息同步
- `pcl_ros` - 点云库 ROS 接口
- `pcl_conversions` - PCL/ROS 转换
- `tf2_ros` - TF2 ROS 绑定
- `Eigen3` - 线性代数库

### 安装

1. **创建 ROS 2 工作空间**（如果还没有）:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **克隆仓库**:
   ```bash
   git clone https://github.com/qza36/pointcloud_fusion.git
   ```

3. **安装依赖项**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **编译软件包**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select pointcloud_fusion
   ```

5. **加载工作空间**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 使用方法

#### 启动节点

```bash
ros2 run pointcloud_fusion fusion_node
```

#### 使用自定义参数

```bash
ros2 run pointcloud_fusion fusion_node --ros-args \
  -p target_frame:=base_link \
  -p lidar_topic:=/BottomLidar/livox/lidar/pointcloud \
  -p rear_lidar_topic:=/TopLidar/livox/lidar/pointcloud \
  -p lidar_frame:=livox_frame_bottom \
  -p rear_lidar_frame:=livox_frame_top
```

### 配置参数

节点支持以下 ROS 2 参数：

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `target_frame` | string | `base_link` | 点云融合的目标参考坐标系 |
| `lidar_topic` | string | `/BottomLidar/livox/lidar/pointcloud` | 第一个激光雷达点云话题 |
| `rear_lidar_topic` | string | `/TopLidar/livox/lidar/pointcloud` | 第二个激光雷达点云话题 |
| `lidar_frame` | string | `livox_frame_bottom` | 第一个激光雷达的坐标系 ID |
| `rear_lidar_frame` | string | `livox_frame_top` | 第二个激光雷达的坐标系 ID |

### 话题

#### 订阅话题
- 第一个激光雷达点云（默认：`/BottomLidar/livox/lidar/pointcloud`）
  - 类型：`sensor_msgs/msg/PointCloud2`
  - 来自第一个激光雷达传感器的点云数据

- 第二个激光雷达点云（默认：`/TopLidar/livox/lidar/pointcloud`）
  - 类型：`sensor_msgs/msg/PointCloud2`
  - 来自第二个激光雷达传感器的点云数据

#### 发布话题
- `/points/fused`
  - 类型：`sensor_msgs/msg/PointCloud2`
  - 目标参考坐标系中的融合点云

### 工作原理

1. **订阅**: 节点订阅来自不同激光雷达传感器的两个点云话题
2. **同步**: 使用近似时间同步来匹配来自两个传感器的点云
3. **变换**: 使用 TF2 将两个点云转换到目标参考坐标系
4. **融合**: 将转换后的点云连接成单个统一的点云
5. **发布**: 将融合后的点云发布到输出话题

### 架构图

```
┌─────────────────┐
│  激光雷达传感器 1 │
│  (底部)         │
└────────┬────────┘
         │ PointCloud2
         ▼
    ┌────────────────────┐
    │                    │
    │  消息过滤器         │◄─────┐
    │  (同步器)          │      │ PointCloud2
    │                    │      │
    └────────┬───────────┘      │
             │                  │
             │            ┌─────┴────────┐
             │            │ 激光雷达传感器 2│
             │            │ (顶部)        │
             ▼            └───────────────┘
    ┌─────────────────────┐
    │  TF2 坐标变换       │
    │  到 target_frame    │
    └────────┬────────────┘
             │
             ▼
    ┌─────────────────────┐
    │  点云融合            │
    └────────┬────────────┘
             │
             ▼
    ┌─────────────────────┐
    │  /points/fused      │
    │  (输出)             │
    └─────────────────────┘
```

### 开发

#### 开发构建
```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_fusion --symlink-install
```

#### 运行测试
```bash
cd ~/ros2_ws
colcon test --packages-select pointcloud_fusion
```

### 故障排除

#### 变换错误
如果遇到变换错误：
- 确保在传感器坐标系和目标坐标系之间正确广播了 TF2 变换
- 检查点云消息的坐标系 ID 是否与 TF 树匹配
- 使用 `ros2 run tf2_ros tf2_echo <source_frame> <target_frame>` 验证变换

#### 同步问题
如果点云无法同步：
- 验证两个激光雷达话题是否正在发布数据
- 检查两个话题的时间戳是否足够接近（在 ~100ms 以内）
- 如有必要，调整同步队列大小

### 许可证

待定：许可证声明

### 维护者

- **维护者**: ZhiangQI
- **邮箱**: an9qi@yandex.com

### 贡献

欢迎贡献！请随时提交问题或拉取请求。
