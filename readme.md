# 无人集群协同导航

```bash
clone co_navi_pkg && catkin_make

pip install pyproj

source /home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/devel/setup.bash
```

rosrun tf view_frames

## ROS Launch List 

支持 视觉 + IMU + GPS 


### 启动方式

#### 低速模式启动
```bash
roslaunch co_navi_pkg co_navi_low.launch
```

#### 中速模式启动
```bash
roslaunch co_navi_pkg co_navi_mid.launch
```

#### 高速模式启动
```bash
roslaunch co_navi_pkg co_navi_high.launch
```

### 系统架构

无人机集群协同导航系统(co_navi_x.launch)
├── 地图生成模块 (map_generator)
├── 多机协调模块 (drones.launch)
├── 单机导航模块 (single_drone.launch)
│ ├── EGO-Planner 路径规划
│ ├── 传感器融合 (IMU + 视觉 + GPS)
│ └── 位置估计与导航
├── 协同定位模块 (cooperative_localization)
└── 距离测量模块 (distance_calculator)