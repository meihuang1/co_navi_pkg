#include <ros/ros.h>
#include <co_navi_pkg/vision_imu_fusion.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_imu_fusion");
  ros::NodeHandle nh("~");
  IMUVisionFusion fusion(nh);
  ros::spin();

  return 0;
}