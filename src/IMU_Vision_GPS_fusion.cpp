#include <ros/ros.h>
#include <co_navi_pkg/vision_imu_GPS_fusion.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_imu_GPS_fusion");
  ros::NodeHandle nh("~");
  IMUVisionGPSFusion fusion(nh);
  ros::spin();

  return 0;
}