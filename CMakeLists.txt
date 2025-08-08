cmake_minimum_required(VERSION 3.0.2)
project(co_navi_pkg)

find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)  # 如果用到 GTSAM

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(eskf_fusion_IG_node src/eskf_fusion_IG_node.cpp)

target_link_libraries(eskf_fusion_IG_node  ${catkin_LIBRARIES})

add_dependencies(eskf_fusion_IG_node  ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})


add_executable(IMU_Lidar_fusion src/IMU_Lidar_fusion.cpp)

target_link_libraries(IMU_Lidar_fusion  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(IMU_Lidar_fusion  ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})


add_executable(IMU_Lidar_fusion_ds src/IMU_Lidar_fusion_ds.cpp)

target_link_libraries(IMU_Lidar_fusion_ds  ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_dependencies(IMU_Lidar_fusion_ds  ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})