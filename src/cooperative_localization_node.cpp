#include "co_navi_pkg/cooperative_localization.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "cooperative_localization_node");
    
    ros::NodeHandle nh("~");  // 私有命名空间
    
    try {
        // 创建协同定位对象
        CooperativeLocalization cooperative_localization(nh);
        
        ROS_INFO("Cooperative Localization Node started successfully!");
        
        // 进入ROS循环
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in Cooperative Localization Node: %s", e.what());
        return 1;
    }
    
    return 0;
}