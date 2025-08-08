#include <ros/ros.h>
#include <co_navi_pkg/lidar_imu_icp_fusion.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "improved_imu_lidar_fusion");
  ros::NodeHandle nh("~");
  IMULidarFusion fusion(nh);
  ros::spin();

  return 0;
}