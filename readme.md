# 无人集群协同导航

```
clone co_navi_pkg && catkin_make

pip install pyproj

source /home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/devel/setup.bash
```

rosrun tf view_frames


## ROS Launch List 
co_navi.launch
├── ego_planner/rviz.launch
│
├── co_navi_pkg/map_drones_generator.launch
│   ├── map_generator/random_forest.launch
│   ├── ego_planner/launch/run_in_sim.launch
│   └──co_navi_pkg/launch/odom_2_gps_imu.launch
│
└── co_navi_pkg/launch/ekf_fusion_multi.launch
    ├── (drone_0)conavi_pkg/launch/ekf_fusion_single.launch
    ├── (drone_1)conavi_pkg/launch/ekf_fusion_single.launch
    ...
    └── (drone_n)conavi_pkg/launch/ekf_fusion_single.launch