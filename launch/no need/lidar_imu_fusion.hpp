#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>


class IMULidarFusion_old
{
public:
  IMULidarFusion_old(ros::NodeHandle &nh) : nh_(nh)
  {
    // 初始化点云
    map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);

    // 订阅全局地图点云
    cloud_map_sub = nh.subscribe("/map_generator/global_cloud", 1, &IMULidarFusion_old::mapCallback, this);

    // 使用message_filters同步里程计和点云
    // odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/drone_0/odometry/filtered", 10));
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/drone_0/global_odom", 10));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/drone_0_pcl_render_node/cloud", 10));

    sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&IMULidarFusion_old::syncCloudOdomCallback, this, _1, _2));

    // 单独订阅IMU用于预测
    imu_sub = nh.subscribe("/drone_0/imu", 100, &IMULidarFusion_old::imuCallback, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/fusion/odom", 10);
    debug_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/filtered_cloud", 1);
    debug_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/filtered_map_cloud", 1);
    // 初始化参数
    predict_pose = Eigen::Matrix4f::Identity();
    last_transform = Eigen::Matrix4f::Identity();
    velocity.setZero();
    position.setZero();
    orientation = Eigen::Quaternionf::Identity();
    map_ready = false;

    // 加载ICP参数
    nh_.param<float>("icp_max_distance", icp_max_distance_, 1.0);
    nh_.param<int>("icp_max_iter", icp_max_iter_, 40);
    nh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 0.25);

    nh_.param<float>("init_x", init_x_, 0.0);
    nh_.param<float>("init_y", init_y_, 0.0);
    nh_.param<float>("init_z", init_z_, 0.0);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub, cloud_map_sub;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  ros::Publisher odom_pub, debug_cloud_pub, debug_map_pub;
  // 定义同步策略
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;

  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr map_with_normals;

  Eigen::Matrix4f predict_pose;
  Eigen::Matrix4f last_transform;
  ros::Time last_imu_time;

  Eigen::Vector3f velocity;
  Eigen::Vector3f position;
  Eigen::Quaternionf orientation;

  bool map_ready;
  float icp_max_distance_;
  int icp_max_iter_;
  float voxel_leaf_size_;
  float init_x_, init_y_, init_z_;

  void syncCloudOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                             const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)

  {
    if (!map_ready)
    {
      ROS_WARN_THROTTLE(1.0, "Map not ready, skipping ICP");
      return;
    }

    // 转换点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *input);

    if (input->empty())
    {
      ROS_WARN("Empty input cloud");
      return;
    }

    // 预处理点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = preprocessCloud(input);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = preprocessCloudUnified(input, voxel_leaf_size_, false);

    // 发布调试点云
    sensor_msgs::PointCloud2 debug_msg;
    pcl::toROSMsg(*filtered, debug_msg);
    debug_msg.header.frame_id = "base_link";
    debug_cloud_pub.publish(debug_msg);

    sensor_msgs::PointCloud2 ros_map_cloud;
    pcl::toROSMsg(*map_cloud, ros_map_cloud);
    ros_map_cloud.header.frame_id = "world";
    debug_map_pub.publish(ros_map_cloud);
    // return filtered;

    // 从里程计获取初始猜测
    // Eigen::Matrix4f init_guess = odomToEigen(odom_msg);

    // 优先使用IMU预测，如果不可用则退回里程计
    Eigen::Matrix4f init_guess;
    if ((ros::Time::now() - last_imu_time).toSec() < 0.5) // IMU预测在最近0.5秒内更新过
    {
      init_guess = predict_pose;
      // ROS_INFO("Using IMU prediction for ICP initialization");
    }
    else
    {
      init_guess = odomToEigen(odom_msg);
      // ROS_INFO("Using odometry pose for ICP initialization");
    }

    // 执行ICP配准
    Eigen::Matrix4f icp_result = runICP(filtered, init_guess);
    std::cout << init_guess << std::endl;
    // 发布结果
    publishOdometry(icp_result);

    // 更新预测位姿
    last_transform = icp_result;
    predict_pose = icp_result;
    position = icp_result.block<3, 1>(0, 3);
    orientation = Eigen::Quaternionf(icp_result.block<3, 3>(0, 0));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloudUnified(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
      float voxel_size,
      bool apply_statistical_filter = true)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>(*input));

    // 1. 可选统计滤波
    if (apply_statistical_filter)
    {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(filtered);
      sor.setMeanK(50);
      sor.setStddevMulThresh(1.0);
      sor.filter(*filtered);
    }

    // 2. 体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.filter(*filtered);

    return filtered;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. 统计滤波去除离群点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered);

    // 2. 体素滤波下采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*filtered);
  }

  Eigen::Matrix4f odomToEigen(const nav_msgs::Odometry::ConstPtr &odom)
  {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0, 3) = odom->pose.pose.position.x;
    pose(1, 3) = odom->pose.pose.position.y;
    pose(2, 3) = odom->pose.pose.position.z;

    Eigen::Quaternionf q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();

    return pose;
  }

  Eigen::Matrix4f runICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, const Eigen::Matrix4f &init_guess)
  {
    // 计算输入点云法线
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);
    ne.setRadiusSearch(0.5);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.compute(*normals);

    // 合并点云和法线
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*input, *normals, *cloud_with_normals);

    // 配置ICP
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(cloud_with_normals);
    icp.setInputTarget(map_with_normals);
    icp.setMaxCorrespondenceDistance(icp_max_distance_);
    icp.setMaximumIterations(icp_max_iter_);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setUseReciprocalCorrespondences(true); // 双向对应提高精度

    // 执行ICP
    pcl::PointCloud<pcl::PointNormal> aligned;
    icp.align(aligned, init_guess);

    if (!icp.hasConverged())
    {
      ROS_WARN("ICP did not converge, using odometry pose");
      return init_guess;
    }

    ROS_INFO_STREAM("ICP fitness: " << icp.getFitnessScore()
                                    << ", iterations: " << icp.getMaximumIterations());

    // 检查ICP结果是否合理
    float translation_diff = (icp.getFinalTransformation().block<3, 1>(0, 3) -
                              init_guess.block<3, 1>(0, 3))
                                 .norm();
    if (translation_diff > 2.0)
    { // 如果位移差异大于2米，认为不可靠
      ROS_WARN("Large ICP deviation (%.2fm), using odometry pose", translation_diff);
      return init_guess;
    }

    return icp.getFinalTransformation();
  }

  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    pcl::fromROSMsg(*msg, *map_cloud);

    // 用统一函数处理地图（注意这里建议关闭统计滤波，仅下采样）
    map_cloud = preprocessCloudUnified(map_cloud, voxel_leaf_size_, false); // false: 地图通常较稳定，可跳过统计滤波

    // 预计算地图法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(map_cloud);
    ne.setRadiusSearch(0.5);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr map_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*map_normals);
    pcl::concatenateFields(*map_cloud, *map_normals, *map_with_normals);

    map_ready = true;
    ROS_INFO("Map ready with %zu points", map_cloud->size());
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
  {
    static bool initialized = false;
    static Eigen::Quaternionf current_q;
    static Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    static Eigen::Vector3f position(init_x_, init_y_, init_z_);
    static ros::Time last_time;

    ros::Time now = imu_msg->header.stamp;

    if (!initialized)
    {
      last_time = now;

      // 初始化姿态为传感器当前姿态
      current_q = Eigen::Quaternionf(
          imu_msg->orientation.w,
          imu_msg->orientation.x,
          imu_msg->orientation.y,
          imu_msg->orientation.z);

      initialized = true;
      return;
    }

    double dt = (now - last_time).toSec();
    last_time = now;

    // 读取角速度
    Eigen::Vector3f gyro(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z);

    // 用四元数更新当前姿态
    Eigen::Quaternionf dq = Eigen::Quaternionf(1, 0.5f * gyro.x() * dt, 0.5f * gyro.y() * dt, 0.5f * gyro.z() * dt);
    current_q = (current_q * dq).normalized();

    // 读取线加速度
    Eigen::Vector3f acc(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);

    // 转换加速度到世界系
    Eigen::Vector3f acc_world = current_q * acc;

    // 去除重力（假设重力方向为Z轴）
    acc_world -= Eigen::Vector3f(0, 0, 9.81f);

    // 更新速度和位置
    velocity += acc_world * dt;
    position += velocity * dt;

    // 保存预测的 pose
    Eigen::Matrix4f pred_pose = Eigen::Matrix4f::Identity();
    pred_pose.block<3, 3>(0, 0) = current_q.toRotationMatrix();
    pred_pose.block<3, 1>(0, 3) = position;
    predict_pose = pred_pose;
    last_imu_time = now;
  }

  void publishOdometry(const Eigen::Matrix4f &pose)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);

    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub.publish(odom);
  }
};
