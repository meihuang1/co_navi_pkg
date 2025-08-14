# 协同导航系统 (Cooperative Navigation System)

## 系统概述
这是一个基于ROS的无人机协同导航系统，支持多无人机在仿真环境中的自主导航和定位。系统集成了IMU、LiDAR、GPS等多种传感器数据，通过多传感器融合算法实现高精度定位。

## 启动方式
```bash
roslaunch co_navi_pkg low_speed/co_navi.launch
```

## 系统架构

### 1. 主启动文件 (`co_navi.launch`)
这是系统的主入口，按顺序启动以下组件：

### 2. 可视化界面
- **RViz节点**: 通过 `ego_planner/rviz.launch` 启动，提供3D可视化界面

### 3. 地图生成器 (`map_drones_generator.launch`)
- **随机森林地图生成器** (`map_generator/random_forest`)
  - 生成42×30×5米的仿真环境
  - 随机生成200个障碍物（半径0.5-0.7米，高度0-3米）
  - 生成200个圆柱形障碍物
  - 感知半径5米，更新频率1Hz

### 4. 多无人机系统 (`drones.launch`)
目前配置了2架无人机：

**无人机0号**:
- 初始位置: (-20.0, -9.0, 1.0)
- 目标位置: (20.0, 9.0, 1.0)

**无人机1号**:
- 初始位置: (-20.0, -7.0, 1.0)  
- 目标位置: (20.0, 7.0, 1.0)

### 5. 单机系统 (`single_drone.launch`)
每架无人机包含以下组件：

#### 5.1 路径规划器
- **EGO-Planner**: 通过 `ego_planner/run_in_sim.launch` 启动
  - 负责无人机路径规划和避障
  - 接收目标点并生成安全轨迹

#### 5.2 传感器数据转换器
- **Odom到GPS/IMU转换器** (`co_navi_pkg/imu_gps_publisher.py`)
  - 将里程计数据转换为GPS和IMU消息格式
  - 设置地理坐标原点 (30.0°N, 120.0°E, 0.0m)

#### 5.3 多传感器融合系统

**ESKF融合节点** (`co_navi_pkg/IMU_GPS_eskf_fusion`):
- 融合IMU和GPS数据
- 噪声参数配置：

**IMU-LiDAR融合节点** (`co_navi_pkg/IMU_Lidar_fusion_ds`):
- 融合IMU和LiDAR点云数据
- ICP配准参数：
  - 最大对应距离: 2.0m
  - 最大迭代次数: 50
  - 体素滤波尺寸: 0.3m

## 话题结构
每架无人机都有独立的话题命名空间：
- `/drone_0/...` - 无人机0号相关话题
- `/drone_1/...` - 无人机1号相关话题

## 关键特性
1. **多传感器融合**: 集成IMU、LiDAR、GPS数据
2. **协同导航**: 支持多无人机同时运行
3. **仿真环境**: 随机生成的森林环境，包含动态障碍物
4. **高精度定位**: 通过ESKF和ICP算法实现厘米级定位精度
5. **实时规划**: 基于EGO-Planner的实时路径规划和避障

## 使用说明
1. 确保ROS环境已正确配置
2. 启动系统: `roslaunch co_navi_pkg low_speed/co_navi.launch`
3. 在RViz中观察无人机运动轨迹和点云数据
4. 可通过修改launch文件参数调整无人机数量和初始位置

## 系统启动流程详解

### 启动顺序
1. **RViz可视化界面** - 提供3D可视化
2. **地图生成器** - 创建仿真环境
3. **多无人机系统** - 启动所有无人机
4. **单机组件** - 每架无人机的完整系统

### 每架无人机的启动组件
1. **EGO-Planner路径规划器**
2. **传感器数据转换器** (Odom→GPS/IMU)
3. **ESKF融合节点** (IMU+GPS)
4. **IMU-LiDAR融合节点** (IMU+LiDAR)

## 参数配置
系统支持通过launch文件配置以下参数：
- 无人机数量
- 初始位置和目标位置
- 地图尺寸和障碍物密度
- 传感器噪声参数
- ICP配准参数

## TODO 
- IMU GPS 融合下，在z轴上 有一点误差 大概在 2m 左右。
