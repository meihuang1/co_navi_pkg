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

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mutex>

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class IMULidarFusion
{
public:
  IMULidarFusion(ros::NodeHandle &nh) : nh_(nh)
  {
    // 初始化点云
    map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);

    // 订阅全局地图点云
    cloud_map_sub = nh.subscribe("/map_generator/global_cloud", 1, &IMULidarFusion::mapCallback, this);

    // 使用message_filters同步里程计和点云
    // odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/drone_0/odometry/filtered", 10));
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/drone_0/global_odom", 10));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/drone_0_pcl_render_node/cloud", 10));

    sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&IMULidarFusion::syncCloudOdomCallback, this, _1, _2));

    // 单独订阅IMU用于预测
    imu_sub = nh.subscribe("/drone_0/imu", 100, &IMULidarFusion::imuCallback, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/fusion/odom", 10);
    debug_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/filtered_cloud", 1);
    debug_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/filtered_map_cloud", 1);
    // 初始化参数
    predict_pose = Eigen::Matrix4f::Identity();
    last_transform_ = Eigen::Matrix4f::Identity();
    velocity_.setZero();
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
    initGTSAM();
    
    // 初始化漂移检测
    accumulated_drift_.setZero();
    last_drift_check_time_ = ros::Time::now();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub, cloud_map_sub;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  ros::Publisher odom_pub, debug_cloud_pub, debug_map_pub;

  // GTSAM预积分和优化器
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_integrator_;
  boost::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  gtsam::imuBias::ConstantBias imu_bias_;

  // 定义同步策略
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;

  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr map_with_normals;

  Eigen::Matrix4f predict_pose;
  Eigen::Matrix4f last_transform_;
  ros::Time last_imu_time_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f position;
  Eigen::Quaternionf orientation;
  // map 及 ros节点信息
  bool map_ready;
  float icp_max_distance_;
  int icp_max_iter_;
  float voxel_leaf_size_;
  float init_x_, init_y_, init_z_;

  int key_index_ = 0; // 当前帧索引
  std::set<gtsam::Key> inserted_keys_;
  std::mutex optimize_mutex_;

  bool debug_model_ = false;
  // 初始化GTSAM
  void initGTSAM()
  {
    // 使用boost::shared_ptr创建参数
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> params =
        gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);

    params->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(0.1, 2);
    params->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(0.01, 2);
    params->integrationCovariance = gtsam::Matrix33::Identity() * 1e-4;

    // 使用boost::make_shared创建预积分器
    imu_integrator_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
        params, gtsam::imuBias::ConstantBias());

    // ISAM2配置
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.relinearizeSkip = 1;
    isam2_ = boost::make_shared<gtsam::ISAM2>(isam_params);
  }
  ros::Time last_optimize_time_;
  double optimize_interval_sec_ = 0.5;
  
  // 漂移检测和修正
  Eigen::Vector3f accumulated_drift_;
  ros::Time last_drift_check_time_;
  double drift_check_interval_ = 5.0;  // 每5秒检测一次漂移
  double max_drift_threshold_ = 10.0;  // 最大允许漂移10米
  bool drift_detected_ = false;

  void syncCloudOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg,
                             const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
  {
    if (!map_ready)
    {
      ROS_WARN_THROTTLE(1.0, "Map not ready, skipping ICP");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = preprocessCloudUnified(input, voxel_leaf_size_, false);

    Eigen::Matrix4f init_guess;
    if ((ros::Time::now() - last_imu_time_).toSec() < 0.5)
    {
      std::cout << "predict_pose " << predict_pose << std::endl;
      init_guess = predict_pose;
      std::cout << "using IMU " << std::endl;
    }

    else
    {
      init_guess = odomToEigen(odom_msg);
      std::cout << "using odom " << std::endl;
    }

    Eigen::Matrix4f icp_result = runICP(filtered, init_guess);
    
    // 漂移检测和修正
    checkAndCorrectDrift(icp_result, init_guess, cloud_msg->header.stamp);

    if ((cloud_msg->header.stamp - last_optimize_time_).toSec() > optimize_interval_sec_)
    {
      optimizeWithGTSAM(icp_result, cloud_msg->header.stamp);
      last_optimize_time_ = cloud_msg->header.stamp;
    }

    predict_pose = last_transform_;
    position = last_transform_.block<3, 1>(0, 3);
    orientation = Eigen::Quaternionf(last_transform_.block<3, 3>(0, 0));
    publishOdometry(last_transform_);
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

    // 2. GTSAM预积分
    double current_time = imu_msg->header.stamp.toSec();
    if (last_imu_time_.toSec() > 0)
    {
      double dt = current_time - last_imu_time_.toSec();
      imu_integrator_->integrateMeasurement(
          gtsam::Vector3(imu_msg->linear_acceleration.x,
                         imu_msg->linear_acceleration.y,
                         imu_msg->linear_acceleration.z),
          gtsam::Vector3(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z),
          dt);
    }
    last_imu_time_ = now;
  }

  void checkAndCorrectDrift(const Eigen::Matrix4f &icp_result, const Eigen::Matrix4f &imu_prediction, const ros::Time &stamp)
  {
    // 计算ICP和IMU预测之间的位置差异
    Eigen::Vector3f icp_pos = icp_result.block<3, 1>(0, 3);
    Eigen::Vector3f imu_pos = imu_prediction.block<3, 1>(0, 3);
    Eigen::Vector3f position_diff = icp_pos - imu_pos;
    
    // 累积漂移
    accumulated_drift_ += position_diff;
    
    // 检查是否需要进行漂移评估
    double time_since_check = (stamp - last_drift_check_time_).toSec();
    if (time_since_check > drift_check_interval_)
    {
      float drift_magnitude = accumulated_drift_.norm();
      
      if (drift_magnitude > max_drift_threshold_)
      {
        ROS_WARN("Large IMU drift detected: %.2f m over %.1f seconds", 
                 drift_magnitude, time_since_check);
        
        // 重置IMU积分状态
        velocity_.setZero();
        predict_pose = icp_result;  // 强制使用LiDAR位置
        
        // 重置GTSAM优化器以避免累积误差
        ROS_WARN("Resetting GTSAM optimizer due to large drift");
        initGTSAM();
        key_index_ = 0;
        
        drift_detected_ = true;
      }
      else if (drift_magnitude > max_drift_threshold_ * 0.5)
      {
        ROS_WARN("Moderate IMU drift detected: %.2f m, applying soft correction", 
                 drift_magnitude);
        
        // 软修正：部分信任LiDAR结果
        float correction_factor = 0.3f;
        Eigen::Vector3f corrected_pos = imu_pos + correction_factor * position_diff;
        predict_pose.block<3, 1>(0, 3) = corrected_pos;
        
        // 减小速度估计的信任度
        velocity_ *= 0.8f;
      }
      else
      {
        drift_detected_ = false;
      }
      
      // 重置累积漂移和时间
      accumulated_drift_.setZero();
      last_drift_check_time_ = stamp;
      
      ROS_INFO_THROTTLE(10.0, "Drift check: %.2f m over %.1f s", 
                        drift_magnitude, time_since_check);
    }
    
    // 如果检测到漂移，增加对LiDAR的信任度
    if (drift_detected_)
    {
      // 当检测到漂移时，更多地依赖ICP结果
      last_transform_ = icp_result;
    }
  }

  template <typename Key, typename Value>
  void safeInsert(boost::shared_ptr<gtsam::ISAM2> isam2, gtsam::Values &values, const Key &key, const Value &value)
  {
    if (!values.exists(key) && !isam2->valueExists(key))
    {
      values.insert(key, value);
      if (debug_model_)
        std::cout << "[safeInsert] Inserted key: " << gtsam::DefaultKeyFormatter(key) << std::endl;
    }
    else
    {
      if (debug_model_)
        std::cout << "[safeInsert] Already exists in ISAM2: " << gtsam::DefaultKeyFormatter(key) << std::endl;
    }
  }

  void optimizeWithGTSAM(const Eigen::Matrix4f &icp_pose, const ros::Time &stamp)
  {
    std::lock_guard<std::mutex> lock(optimize_mutex_); // 🔒 加锁防并发

    try {
      // 初始化第一帧的先验约束
      if (key_index_ == 0)
      {
        // 位姿先验约束
        gtsam::noiseModel::Diagonal::shared_ptr pose_prior_noise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
        gtsam::Pose3 lidar_pose = eigenToGtsamPose(icp_pose);
        graph_.addPrior(X(0), lidar_pose, pose_prior_noise);
        initial_values_.insert(X(0), lidar_pose);

        // 速度先验约束（更强的约束）
        gtsam::noiseModel::Isotropic::shared_ptr velocity_prior_noise =
            gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
        gtsam::Vector3 prior_velocity(0.0, 0.0, 0.0);
        graph_.addPrior(V(0), prior_velocity, velocity_prior_noise);
        initial_values_.insert(V(0), prior_velocity);

        // IMU bias 先验约束（更强的约束）
        gtsam::noiseModel::Diagonal::shared_ptr bias_prior_noise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4).finished());
        gtsam::imuBias::ConstantBias prior_bias;
        graph_.addPrior(B(0), prior_bias, bias_prior_noise);
        initial_values_.insert(B(0), prior_bias);

        // 更新 ISAM2
        isam2_->update(graph_, initial_values_);
        
        // 获取优化结果
        gtsam::Values result = isam2_->calculateEstimate();
        last_transform_ = gtsamPoseToEigen(result.at<gtsam::Pose3>(X(0)));
        velocity_ = result.at<gtsam::Vector3>(V(0)).cast<float>();
        imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(0));
        
        key_index_++;
        return;
      }

      // 后续帧处理
      gtsam::Pose3 lidar_pose = eigenToGtsamPose(icp_pose);
      
      // 清空临时变量（不清空历史）
      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;

      int i = key_index_ - 1; // 前一帧
      int j = key_index_;     // 当前帧

      // 添加当前帧的LiDAR观测约束
      gtsam::noiseModel::Diagonal::shared_ptr lidar_noise =
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());
      new_factors.addPrior(X(j), lidar_pose, lidar_noise);
      new_values.insert(X(j), lidar_pose);

      // 添加 IMU 预积分因子（如果有足够的 IMU 数据）
      if (imu_integrator_->deltaTij() > 0.01) // 至少10ms的积分时间
      {
        // 获取前一帧状态
        gtsam::Values current_estimate = isam2_->calculateEstimate();
        
        if (current_estimate.exists(X(i)) && current_estimate.exists(V(i)) && current_estimate.exists(B(i)))
        {
          gtsam::Pose3 prev_pose = current_estimate.at<gtsam::Pose3>(X(i));
          gtsam::Vector3 prev_velocity = current_estimate.at<gtsam::Vector3>(V(i));
          gtsam::imuBias::ConstantBias prev_bias = current_estimate.at<gtsam::imuBias::ConstantBias>(B(i));
          
          gtsam::NavState prev_state(prev_pose, prev_velocity);
          gtsam::NavState predicted_state = imu_integrator_->predict(prev_state, prev_bias);

          // 添加IMU因子
          new_factors.add(gtsam::CombinedImuFactor(
              X(i), V(i), X(j), V(j), B(i), B(j), *imu_integrator_));

          // 添加初始值估计
          new_values.insert(V(j), predicted_state.v());
          new_values.insert(B(j), prev_bias); // bias变化缓慢

          // 添加 bias 连续性约束（随机游走模型）
          gtsam::noiseModel::Diagonal::shared_ptr bias_model =
              gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5).finished());
          new_factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
              B(i), B(j), gtsam::imuBias::ConstantBias(), bias_model));
        }
        else
        {
          ROS_WARN("Previous frame state not available, using fallback");
          new_values.insert(V(j), gtsam::Vector3(0, 0, 0));
          new_values.insert(B(j), imu_bias_);
        }
      }
      else
      {
        // 没有足够的IMU数据，使用零速度和当前bias
        new_values.insert(V(j), gtsam::Vector3(0, 0, 0));
        new_values.insert(B(j), imu_bias_);
        
        // 添加弱的速度约束
        gtsam::noiseModel::Isotropic::shared_ptr vel_noise = 
            gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
        new_factors.addPrior(V(j), gtsam::Vector3(0, 0, 0), vel_noise);
      }

      // 更新 ISAM2
      isam2_->update(new_factors, new_values);
      
      // 获取优化结果
      gtsam::Values result = isam2_->calculateEstimate();
      
      if (result.exists(X(j)))
        last_transform_ = gtsamPoseToEigen(result.at<gtsam::Pose3>(X(j)));
      else
        last_transform_ = icp_pose;

      if (result.exists(V(j)))
        velocity_ = result.at<gtsam::Vector3>(V(j)).cast<float>();
      else
        velocity_.setZero();

      if (result.exists(B(j)))
        imu_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(j));

      // 重置IMU积分器
      imu_integrator_->resetIntegrationAndSetBias(imu_bias_);

      key_index_++;
      
      // 限制关键帧数量，避免内存和计算问题
      if (key_index_ > 50) {
        // 可以实现滑动窗口或边缘化
        ROS_WARN_THROTTLE(5.0, "Consider implementing sliding window optimization");
      }
      
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      ROS_ERROR("GTSAM optimization failed: %s", e.what());
      ROS_ERROR("Using ICP result as fallback");
      last_transform_ = icp_pose;
      velocity_.setZero();
      
      // 重置优化器状态
      initGTSAM();
      key_index_ = 0;
      
    } catch (const std::exception& e) {
      ROS_ERROR("Unexpected error in GTSAM optimization: %s", e.what());
      last_transform_ = icp_pose;
    }
  }

  // 将Eigen矩阵转换为gtsam::Pose3
  gtsam::Pose3 eigenToGtsamPose(const Eigen::Matrix4f &mat)
  {
    return gtsam::Pose3(
        gtsam::Rot3(mat.block<3, 3>(0, 0).cast<double>()),
        gtsam::Point3(mat(0, 3), mat(1, 3), mat(2, 3)));
  }

  // 将gtsam::Pose3转换为Eigen矩阵
  Eigen::Matrix4f gtsamPoseToEigen(const gtsam::Pose3 &pose)
  {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat.block<3, 3>(0, 0) = pose.rotation().matrix().cast<float>();
    mat.block<3, 1>(0, 3) = pose.translation().cast<float>();
    return mat;
  }
  void publishOdometry(const Eigen::Matrix4f &pose)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";

    // 位置
    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);

    // 姿态
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 速度 - 添加安全检查
    try
    {
      if (initial_values_.exists(V(1)))
      {
        gtsam::Velocity3 vel = initial_values_.at<gtsam::Velocity3>(V(1));
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
      }
      else
      {
        odom.twist.twist.linear.x = velocity_.x();
        odom.twist.twist.linear.y = velocity_.y();
        odom.twist.twist.linear.z = velocity_.z();
      }
    }
    catch (gtsam::ValuesKeyDoesNotExist &e)
    {
      ROS_WARN("Velocity not available in values: %s", e.what());
      odom.twist.twist.linear.x = velocity_.x();
      odom.twist.twist.linear.y = velocity_.y();
      odom.twist.twist.linear.z = velocity_.z();
    }

    odom_pub.publish(odom);
  }
};
