// eskf_fusion_node.cpp  — Minimal Error‑State EKF fusing IMU + GPS (ROS Noetic)
// -----------------------------------------------------------------------------
//   * Subscribes   : /drone_0/imu   (sensor_msgs/Imu, 100 Hz)
//                    /drone_0/gps  (sensor_msgs/NavSatFix, 10 Hz)
//   * Publishes    : /odometry/filtered (nav_msgs/Odometry)
//   * Publishes TF : drone_0/odom  ->  drone_0/base_link
// -----------------------------------------------------------------------------
//  QUICK BUILD (inside catkin workspace)
//   add_executable(eskf_fusion_node src/eskf_fusion_node.cpp)
//   target_link_libraries(eskf_fusion_node ${catkin_LIBRARIES})
// -----------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Dense>
#include <deque>
#include <cmath>
#include <co_navi_pkg/hpFnc.hpp>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Vec15 = Eigen::Matrix<double, 15, 1>;

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kG = 9.80665; // gravity (m/s^2)

// ----------------------------------------------------------------------------
// Helper — skips full WGS->ENU conversion; here we treat (lat,lon) offset linearly
// for demo. Replace with rigorous geodetic conversion in production.
inline Vec3 lla2enu(double lat0, double lon0, const sensor_msgs::NavSatFix &fix)
{
    const double Re = 6.378137e6; // Earth radius (m)
    double dLat = (fix.latitude - lat0) * kDeg2Rad;
    double dLon = (fix.longitude - lon0) * kDeg2Rad;
    double x = Re * dLon * std::cos(lat0 * kDeg2Rad); // East
    double y = Re * dLat;                             // North
    double z = fix.altitude;                          // Up  (no offset)
    return {x, y, z};
}

// ----------------------------------------------------------------------------
class ESKFNode
{
public:
    ESKFNode() : nh_("~"), first_imu_(true), enu_origin_set_(false)
    {
        // -------------- parameters --------------------
        int drone_id;
        nh_.param("drone_id", drone_id, 0); // 默认是 0

        std::string ns = "drone_" + std::to_string(drone_id);

        frame_id_ = ns + "/base_link";
        odom_frame_ = ns + "/odom";
        imu_topic_ = "/" + ns + "/imu";
        gps_topic_ = "/" + ns + "/gps";
        mag_topic_ = "/" + ns + "/mag";
        vgps_topic_ = "/" + ns + "/gps_vel";
        odom_vis_topic_ = "/" + ns + "_visual_slam/odom";
        repub_odom_topic_ = "/" + ns + "/odom_fused_IG";
        repub_g_odom_topic_ = "/" + ns + "/odom_fused_IG_global";
        repub_gpos_topic_ = "/" + ns + "/pose_fused_IG";

        double sigma_g, sigma_a, sigma_p, sigma_m, sigma_vgps;
        nh_.param("sigma_g", sigma_g, 0.005); // rad/s ^½
        nh_.param("sigma_a", sigma_a, 0.05);  // m/s²
        nh_.param("sigma_p", sigma_p, 0.3);   // m (GPS std)
        nh_.param("sigma_m", sigma_m, 5e-6);
        nh_.param("sigma_vgps", sigma_vgps, 0.25);

        double origin_lat, origin_lon, origin_alt;
        nh_.param("origin_lat", origin_lat, 30.0);  // rad/s ^½
        nh_.param("origin_lon", origin_lon, 120.0); // m/s²
        nh_.param("origin_alt", origin_alt, 0.0);   // m (GPS std)
        nh_.param("init_x", init_x_, 0.0);
        nh_.param("init_y", init_y_, 0.0);
        nh_.param("init_z", init_z_, 0.0);
        nh_.param<std::string>("run_mode", mode_, "low");

        // ---- noise covariances
        Qd_.setZero();
        Qd_.block<3, 3>(0, 0) = Mat3::Identity() * sigma_g * sigma_g; // gyro
        Qd_.block<3, 3>(3, 3) = Mat3::Identity() * sigma_a * sigma_a; // acc
        Qd_.block<3, 3>(9, 9) = Mat3::Identity() * 1e-6;              // bg random walk
        Qd_.block<3, 3>(12, 12) = Mat3::Identity() * 1e-5;            // ba random walk

        R_gps_ = Mat3::Identity() * sigma_p * sigma_p;
        R_gps_vel_ = Mat3::Identity() * sigma_vgps * sigma_vgps;
        // ---- init state / covariance
        q_wb_ = Quat::Identity();
        v_w_.setZero();
        p_w_.setZero();
        bg_.setZero();
        ba_.setZero();

        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 1e-4;   // 角度
        P_.block<3, 3>(3, 3) *= 1e-2;   // 速度
        P_.block<3, 3>(6, 6) *= 1e-2;   // 位置
        P_.block<3, 3>(9, 9) *= 1e-6;   // gyro bias
        P_.block<3, 3>(12, 12) *= 1e-5; // acc bias

        // ---- ROS pubs/subs
        sub_imu_ = nh_.subscribe(imu_topic_, 200, &ESKFNode::imuCallback, this, ros::TransportHints().tcpNoDelay());
        sub_gps_ = nh_.subscribe(gps_topic_, 50, &ESKFNode::gpsCallback, this, ros::TransportHints().tcpNoDelay());
        sub_mag_ = nh_.subscribe(mag_topic_, 50, &ESKFNode::magCallback, this, ros::TransportHints().tcpNoDelay());
        sub_vgps_ = nh_.subscribe(vgps_topic_, 50, &ESKFNode::vGPSCallback, this, ros::TransportHints().tcpNoDelay());
        sub_odom_vis_ = nh_.subscribe(odom_vis_topic_, 50, &ESKFNode::odomVisCallback, this, ros::TransportHints().tcpNoDelay());
        
        array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("double_infos", 10);
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>(repub_odom_topic_, 50);
        pub_global_odom_ = nh_.advertise<nav_msgs::Odometry>(repub_g_odom_topic_, 50);
        pub_global_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(repub_gpos_topic_, 50);
    }

private:
    // --------------------------- state members ------------------------------
    Quat q_wb_;    // world->body
    Vec3 v_w_;     // vel in world
    Vec3 p_w_;     // pos in world (ENU)
    Vec3 bg_, ba_; // biases
    Mat15 P_;
    Mat15 Qd_;
    Mat3 R_gps_;
    Mat3 R_gps_vel_;
    Vec3 ang_;
    // ---------------------------- ZUPT 零偏检测 --------------------------------

    bool is_static_ = false;
    const double GYRO_STATIC_THRESH = 0.03; // rad/s，陀螺仪静止阈值
    const double ACC_STATIC_THRESH = 0.1;   // 加速度静止阈值 (m/s²)
    const int STATIC_COUNT_THRESHOLD = 20;  // 连续多少帧认为静止
    int static_counter_ = 0;
    std::string mode_;
    // ----------------------------filter members --------------------------------
    std::deque<Vec3> acc_buffer_;
    const int ACC_BUFFER_SIZE = 30;

    Vec3 v_filtered_ = Vec3::Zero();
    double v_filter_alpha_ = 0.1; // 越接近1越平滑，建议 0.95 ~ 0.99

    Vec3 ang_filtered_;              // 初始化为 0
    double ang_filter_alpha_ = 0.98; // 或者 0.9 ~ 0.98 之间调节

    Vec3 visPos_, visVel_;

    // --------------------------- ROS members --------------------------------
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_, sub_gps_, sub_mag_, sub_vgps_, sub_odom_vis_;
    ros::Publisher pub_odom_, array_pub_, pub_global_pose_, pub_global_odom_;
    std_msgs::Float64MultiArray msg;
    tf2_ros::TransformBroadcaster tf_pub_;

    std::string frame_id_, odom_frame_, imu_topic_, gps_topic_, mag_topic_,
        vgps_topic_, repub_odom_topic_, repub_gpos_topic_, repub_g_odom_topic_, odom_vis_topic_;

    ros::Time prev_imu_stamp_;
    bool first_imu_;

    bool init_bias_done_ = false;
    int init_count_ = 0;
    const int INIT_COUNT_MAX = 50;
    Vec3 acc_sum_ = Vec3::Zero();

    bool init_q_done_ = false;
    Vec3 prev_acc_;
    // ENU origin
    double origin_lat_, origin_lon_;
    bool enu_origin_set_;

    double init_x_, init_y_, init_z_;
    inline Mat3 SkewSymmetric(const Vec3 &v)
    {
        Mat3 m;
        m << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return m;
    }
    // -------------------------- callbacks -----------------------------------
    void imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        static int init_count = 0;
        static Vec3 acc_sum = Vec3::Zero();

        Vec3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Vec3 ang(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Quat q_imu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        ang_ = ang;
        // === 第一步：初始化加速度 bias（静止平均法）===
        if (!init_bias_done_)
        {
            acc_sum += acc;
            init_count++;
            if (init_count >= 50)
            {
                Vec3 acc_avg = acc_sum / init_count;
                ba_ = acc_avg - Vec3(0, 0, kG);
                init_bias_done_ = true;
                ROS_INFO_STREAM("Initial accelerometer bias estimated: " << ba_.transpose());
            }
            return;
        }

        // === 第二步：初始化姿态（只执行一次）===
        if (!init_q_done_)
        {
            q_wb_ = q_imu.normalized(); // 用 IMU 提供的 orientation 初始化
            init_q_done_ = true;
            ROS_INFO_STREAM("Initial orientation set from IMU: "
                            << q_wb_.w() << ", " << q_wb_.x() << ", " << q_wb_.y() << ", " << q_wb_.z());
            prev_imu_stamp_ = msg->header.stamp;
            first_imu_ = false;
            return;
        }

        // === 第三步：正常执行 predict ===
        if (first_imu_)
        {
            prev_imu_stamp_ = msg->header.stamp;
            first_imu_ = false;
            return;
        }

        double dt = 0.01;
        prev_imu_stamp_ = msg->header.stamp;

        predict(ang, acc, q_imu, dt); // 使用积分的姿态更新
        publishOdom(msg->header.stamp);
    }

    void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
    {
        if (msg->status.status < 0)
            return; // no fix

        if (!enu_origin_set_)
        {
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            enu_origin_set_ = true;
        }

        Vec3 pos_meas = lla2enu(origin_lat_, origin_lon_, *msg);
        correctGps(pos_meas);
    }

    void magCallback(const sensor_msgs::MagneticFieldConstPtr &msg)
    {
        Vec3 mag_meas(msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z);
        // correctMag(mag_meas);
    }

    void vGPSCallback(const geometry_msgs::TwistStampedConstPtr &msg)
    {
        Vec3 vel_meas(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
        correctGpsVel(vel_meas);
    }


    void odomVisCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
      Vec3 pos(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
      Vec3 vel(msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);
  
      visPos_ = pos;
      visVel_ = vel;
    }

    void correctMag(const Vec3 &mag_meas)
    {
        // 1. 世界系地磁方向固定为 ENU x 轴（东向）
        static const Vec3 mag_world(1.0, 0.0, 0.0); // or Vec3(0.707, 0.707, 0.0) if you want NE

        // 2. 预测值：将世界磁场转到机体系（用当前估计姿态）
        Vec3 mag_pred = q_wb_.inverse() * mag_world;

        // 3. 残差：单位化后比较
        Vec3 res = mag_meas.normalized() - mag_pred.normalized();

        // 4. 雅可比 H（磁场只对姿态敏感，其他状态偏导为 0）
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = -SkewSymmetric(mag_pred); // 更直观，数值也更稳定

        // 5. 观测协方差（建议小一点更有约束力）
        Mat3 R_mag = Mat3::Identity() * 0.01;

        // 6. 卡尔曼增益
        Mat3 S = H * P_ * H.transpose() + R_mag;
        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();

        // 7. 状态更新
        Vec15 dx = K * res;
        Vec3 dtheta = dx.segment<3>(0);
        Quat dq(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
        q_wb_ = (q_wb_ * dq).normalized();
        v_w_ += dx.segment<3>(3);
        p_w_ += dx.segment<3>(6);
        bg_ += dx.segment<3>(9);
        ba_ += dx.segment<3>(12);

        // 8. 协方差更新
        Mat15 I = Mat15::Identity();
        P_ = (I - K * H) * P_;
    }

    // --------------------------- acc filter 均值滤波器 ---------------------------------
    Vec3 filterAcc(const Vec3 &acc_raw)
    {
        acc_buffer_.push_back(acc_raw);
        if (acc_buffer_.size() > ACC_BUFFER_SIZE)
            acc_buffer_.pop_front();

        Vec3 acc_sum(0, 0, 0);
        for (auto &a : acc_buffer_)
            acc_sum += a;
        return acc_raw - acc_sum / acc_buffer_.size(); // 去掉最近窗口的平均值
    }
    // ------------------------------ core EKF ---------------------------------
    void predict(const Vec3 &ang_m, const Vec3 &acc_m, const Quat q_imu, double dt)
    {
        // Remove biases
        Vec3 ang = ang_m - bg_;
        Vec3 acc = acc_m - ba_;

        // 滤波角速度
        ang_filtered_ = ang_filter_alpha_ * ang_filtered_ + (1.0 - ang_filter_alpha_) * ang;
        Vec3 ang_smooth = ang_filtered_;

        // 姿态更新（一级欧拉积分）
        // Quat dq(1, 0.5 * ang_smooth.x() * dt,
        //         0.5 * ang_smooth.y() * dt,
        //         0.5 * ang_smooth.z() * dt);
        // q_wb_ = (q_wb_ * dq).normalized();
        q_wb_ = q_wb_.slerp(0.95, q_imu).normalized();
        // q_wb_ = q_imu;
        // 世界坐标加速度（考虑重力 + 平滑滤波）
        Vec3 g_w(0, 0, -kG);
        // Vec3 acc_w = q_wb_ * acc + g_w;
        Vec3 acc_w = acc + g_w;
        // acc_w = filterAcc(acc_w);
        // std::cout << "x() " << acc_w.x() << " y() " << acc_w.y() << " z() " << acc_w.z() << std::endl;
        // 判断是否静止
        bool gyro_static = ang_m.norm() < GYRO_STATIC_THRESH;
        bool acc_static = (acc_m - Vec3(0, 0, kG)).norm() < ACC_STATIC_THRESH;

        if (gyro_static && acc_static)
        {
            static_counter_++;
            if (static_counter_ > STATIC_COUNT_THRESHOLD)
            {
                is_static_ = true;
                // 静止检测后归零
                if (static_counter_ == STATIC_COUNT_THRESHOLD + 1)
                {
                    // v_w_.setZero();
                    ROS_INFO("ZUPT: Velocity zeroed due to static state.");
                }
            }
        }
        else
        {
            static_counter_ = 0;
            is_static_ = false;
        }
        // 速度和位置更新
        // if (!is_static_)
        // {
        v_w_ += acc_w * dt;
        // }
        // else
        // {
        //     v_w_ = v_w_ * 0.1; // 缓慢减速，防止跳变
        //     acc_w.setZero();
        // }

        // 滤波速度
        // v_filtered_ = v_filter_alpha_ * v_filtered_ + (1.0 - v_filter_alpha_) * v_w_;
        p_w_ += v_w_ * dt + 0.5 * acc_w * dt * dt;

        // 协方差简单传播
        P_.block<3, 3>(0, 0) += Mat3::Identity() * 1e-5 * dt;   // rot
        P_.block<3, 3>(3, 3) += Mat3::Identity() * 1e-2 * dt;   // vel
        P_.block<3, 3>(6, 6) += Mat3::Identity() * 1e-3 * dt;   // pos
        P_.block<3, 3>(9, 9) += Mat3::Identity() * 1e-6 * dt;   // bg
        P_.block<3, 3>(12, 12) += Mat3::Identity() * 1e-5 * dt; // ba
    }

    void correctGps(const Vec3 &pos_meas)
    {
        if (!enu_origin_set_)
            return;
        // Measurement model: z = p + n
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 6) = Mat3::Identity();

        Vec3 res = pos_meas - p_w_;
        Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R_gps_;
        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();
        Vec15 dx = K * res;

        // Inject error
        p_w_ += dx.segment<3>(6);
        v_w_ += dx.segment<3>(3);
        bg_ += dx.segment<3>(9);
        ba_ += dx.segment<3>(12);
        // small‑angle rot update
        Vec3 dtheta = dx.segment<3>(0);
        Quat dq(1, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
        q_wb_ = (q_wb_ * dq).normalized();

        // Covariance update
        Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I - K * H) * P_;
    }

    void correctGpsVel(const Vec3 &vel_meas)
    {
        // Measurement model: z = v + n
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 3) = Mat3::Identity(); // 对速度部分敏感

        Vec3 res = vel_meas - v_w_; // 残差（GPS速度 - 当前估计速度）

        Mat3 S = H * P_ * H.transpose() + R_gps_vel_;
        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose() * S.inverse();

        Vec15 dx = K * res;

        // 更新状态
        Vec3 dtheta = dx.segment<3>(0);
        Quat dq(1, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
        q_wb_ = (q_wb_ * dq).normalized();
        v_w_ += dx.segment<3>(3);
        p_w_ += dx.segment<3>(6);
        bg_ += dx.segment<3>(9);
        ba_ += dx.segment<3>(12);

        // 协方差更新
        Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
        P_ = (I - K * H) * P_;
    }
    // -------------------------- ROS Publish ----------------------------------
    void publishOdom(const ros::Time &stamp)
    {
        nav_msgs::Odometry odom;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = frame_id_;
        odom.header.stamp = stamp;

        odom.pose.pose.position.x = p_w_.x();
        odom.pose.pose.position.y = p_w_.y();
        odom.pose.pose.position.z = p_w_.z();
        odom.pose.pose.orientation.x = q_wb_.x();
        odom.pose.pose.orientation.y = q_wb_.y();
        odom.pose.pose.orientation.z = q_wb_.z();
        odom.pose.pose.orientation.w = q_wb_.w();

        // odom.twist.twist.linear.x = v_filtered_.x();
        // odom.twist.twist.linear.y = v_filtered_.y();
        // odom.twist.twist.linear.z = v_filtered_.z();
        Eigen::Vector3d outVel = v_w_;
        if(mode_ == "low"){
            outVel = outputProcessing(visVel_, v_w_, mode_, true);
        }

        odom.twist.twist.linear.x = outVel.x();
        odom.twist.twist.linear.y = outVel.y();
        odom.twist.twist.linear.z = outVel.z();

        odom.twist.twist.angular.x = ang_.x();
        odom.twist.twist.angular.y = ang_.y();
        odom.twist.twist.angular.z = ang_.z();
        
        pub_odom_.publish(odom);

        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = odom_frame_;
        pose.header.stamp = stamp;

        pose.pose.position.x = p_w_.x() + init_x_;
        pose.pose.position.y = p_w_.y() + init_y_;
        pose.pose.position.z = p_w_.z();

        // === 姿态 ===
        pose.pose.orientation.x = q_wb_.x();
        pose.pose.orientation.y = q_wb_.y();
        pose.pose.orientation.z = q_wb_.z();
        pose.pose.orientation.w = q_wb_.w();

        // 发布
        pub_global_pose_.publish(pose);

        nav_msgs::Odometry g_odom;
        g_odom.header.frame_id = odom_frame_;
        g_odom.child_frame_id = frame_id_;
        g_odom.header.stamp = stamp;

        g_odom.pose.pose.position.x = p_w_.x() + init_x_;
        g_odom.pose.pose.position.y = p_w_.y() + init_y_;
        g_odom.pose.pose.position.z = p_w_.z();
        g_odom.pose.pose.orientation.x = q_wb_.x();
        g_odom.pose.pose.orientation.y = q_wb_.y();
        g_odom.pose.pose.orientation.z = q_wb_.z();
        g_odom.pose.pose.orientation.w = q_wb_.w();

        // g_odom.twist.twist.linear.x = v_filtered_.x();
        // g_odom.twist.twist.linear.y = v_filtered_.y();
        // g_odom.twist.twist.linear.z = v_filtered_.z();

        g_odom.twist.twist.linear.x = outVel.x();
        g_odom.twist.twist.linear.y = outVel.y();
        g_odom.twist.twist.linear.z = outVel.z();

        g_odom.twist.twist.angular.x = ang_.x();
        g_odom.twist.twist.angular.y = ang_.y();
        g_odom.twist.twist.angular.z = ang_.z();

        pub_global_odom_.publish(g_odom);

        // Broadcast TF
        geometry_msgs::TransformStamped tfm;
        tfm.header = odom.header;
        // tf_msg.header.stamp = ros::Time::now();
        tfm.header.frame_id = "drone_0/odom";     // 父坐标系
        tfm.child_frame_id = "drone_0/base_link"; // 子坐标系

        tfm.transform.translation.x = p_w_.x();
        tfm.transform.translation.y = p_w_.y();
        tfm.transform.translation.z = p_w_.z();
        tfm.transform.rotation = odom.pose.pose.orientation;
        tf_pub_.sendTransform(tfm);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eskf_fusion_node");
    ESKFNode node;
    ros::spin();
    return 0;
}
