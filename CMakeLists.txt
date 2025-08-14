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

add_executable(IMU_GPS_eskf_fusion src/IMU_GPS_eskf_fusion.cpp)
target_link_libraries(IMU_GPS_eskf_fusion  ${catkin_LIBRARIES})
add_dependencies(IMU_GPS_eskf_fusion  ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})

add_executable(IMU_Vision_fusion src/IMU_Vision_fusion.cpp)
target_link_libraries(IMU_Vision_fusion  ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
add_dependencies(IMU_Vision_fusion  ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})

add_executable(IMU_Vision_GPS_fusion src/IMU_Vision_GPS_fusion.cpp)
target_link_libraries(IMU_Vision_GPS_fusion  ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
add_dependencies(IMU_Vision_GPS_fusion  ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})