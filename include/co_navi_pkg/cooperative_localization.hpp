#ifndef COOPERATIVE_LOCALIZATION_HPP
#define COOPERATIVE_LOCALIZATION_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <boost/bind.hpp>
#include <deque>

class CooperativeLocalization
{
public:
    CooperativeLocalization(ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.param<int>("drone_id", drone_id_, 0);
        nh_.param<int>("total_drones", total_drones_, 4);
        nh_.param<std::string>("mode", mode_, "2d"); // "2d" 或 "3d"

        std::string topic_prefix = "/drone_" + std::to_string(drone_id_);

        drone_positions_.assign(total_drones_, Eigen::Vector3d::Zero());
        distances2d_.assign(total_drones_, 0.0);
        distances3d_.assign(total_drones_, 0.0);

        // 订阅其他无人机 odom
        for (int i = 0; i < total_drones_; ++i)
        {
            if (i == drone_id_)
                continue;
            std::string sub_topic = "/drone_" + std::to_string(i) + "_visual_slam/odom";
            ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(
                sub_topic, 1,
                boost::bind(&CooperativeLocalization::dronePoseCallback, this, _1, i));
            drone_subs_.push_back(sub);
        }

        // 订阅 2D 距离
        for (int i = 0; i < total_drones_; ++i)
        {
            if (i == drone_id_)
                continue;
            std::string dist2d_topic = "/dist2d_" + std::to_string(drone_id_) + "_" + std::to_string(i);
            ros::Subscriber sub2d = nh_.subscribe<std_msgs::Float64>(
                dist2d_topic, 1,
                boost::bind(&CooperativeLocalization::distance2DCallback, this, _1, i));
            distance2d_subs_.push_back(sub2d);

            std::string dist3d_topic = "/dist3d_" + std::to_string(drone_id_) + "_" + std::to_string(i);
            ros::Subscriber sub3d = nh_.subscribe<std_msgs::Float64>(
                dist3d_topic, 1,
                boost::bind(&CooperativeLocalization::distance3DCallback, this, _1, i));
            distance3d_subs_.push_back(sub3d);
        }

        // 订阅 IMU 积分结果（用于 Z）
        imu_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            topic_prefix + "/imu_integrator_res", 1,
            &CooperativeLocalization::imuCallback, this);

        fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(
            topic_prefix + "/odom_fused_cooperative_pose", 10);

        ROS_INFO("Cooperative Localization initialized for drone_%d in %s mode", drone_id_, mode_.c_str());
    }

private:
    ros::NodeHandle nh_;
    int drone_id_;
    int total_drones_;
    std::string mode_; // "2d" 或 "3d"

    std::vector<ros::Subscriber> drone_subs_;
    std::vector<ros::Subscriber> distance2d_subs_;
    std::vector<ros::Subscriber> distance3d_subs_;
    ros::Subscriber imu_sub_;
    ros::Publisher fused_odom_pub_;

    std::vector<Eigen::Vector3d> drone_positions_;
    std::vector<double> distances2d_;
    std::vector<double> distances3d_;
    double my_z_ = 0.0;

    // ----------------- 回调 -----------------
    void imuCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        my_z_ = msg->pose.pose.position.z;
        tryTrilateration();
    }

    void dronePoseCallback(const nav_msgs::Odometry::ConstPtr &msg, int id)
    {
        drone_positions_[id] = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        tryTrilateration();
    }

    void distance2DCallback(const std_msgs::Float64::ConstPtr &msg, int id)
    {
        distances2d_[id] = msg->data;
        tryTrilateration();
    }

    void distance3DCallback(const std_msgs::Float64::ConstPtr &msg, int id)
    {
        distances3d_[id] = msg->data;
        tryTrilateration();
    }

    // ----------------- Trilateration 主逻辑 -----------------
    void tryTrilateration()
    {
        if (!hasAllData())
            return;

        // 检查高度差，如果太小就强制用2D模式
        double max_height_diff = 0.0;
        double min_height = 1000.0, max_height = -1000.0;

        for (int i = 0; i < total_drones_; ++i)
        {
            if (i == drone_id_)
                continue;
            if (drone_positions_[i].norm() > 1e-6)
            {
                min_height = std::min(min_height, drone_positions_[i].z());
                max_height = std::max(max_height, drone_positions_[i].z());
            }
        }

        max_height_diff = max_height - min_height;

        // 如果高度差小于阈值，强制使用2D模式
        if (max_height_diff < 2.0)
        { // 2米阈值
            if (mode_ == "3d")
            {
                ROS_WARN_THROTTLE(2.0, "Height difference too small (%.2f m), forcing 2D mode", max_height_diff);
            }
            // 强制使用2D trilateration
            Eigen::Vector3d est = trilateration2D(my_z_);
            if (!est.isZero(0))
            {
                publishFusedPose(est);
            }
            return;
        }

        // 正常3D trilateration
        Eigen::Vector3d est;
        if (mode_ == "3d")
        {
            est = trilateration3D();
        }
        else
        {
            est = trilateration2D(my_z_);
        }

        if (est.isZero(0))
            return;
        publishFusedPose(est);
    }

    bool hasAllData()
    {
        int valid_anchors = 0;
        for (int i = 0; i < total_drones_; ++i)
        {
            if (i == drone_id_)
                continue;
            if (mode_ == "3d")
            {
                if (drone_positions_[i].norm() > 1e-6 && distances3d_[i] > 1e-6)
                    valid_anchors++;
            }
            else
            {
                if (drone_positions_[i].norm() > 1e-6 && distances2d_[i] > 1e-6)
                    valid_anchors++;
            }
        }

        if ((mode_ == "3d" && valid_anchors < 4) || (mode_ == "2d" && valid_anchors < 3))
        {
            ROS_WARN_THROTTLE(1.0, "%s mode needs enough anchors, got %d", mode_.c_str(), valid_anchors);
            return false;
        }

        if (mode_ == "2d" && std::abs(my_z_) < 1e-6)
        {
            ROS_WARN_THROTTLE(1.0, "2D mode needs IMU Z data");
            return false;
        }

        return true;
    }

    // ----------------- 2D Trilateration -----------------
    Eigen::Vector3d trilateration2D(double z_me)
    {
        // 使用线性最小二乘
        struct Anchor
        {
            Eigen::Vector3d P;
            double r;
        };
        std::vector<Anchor> anchors;

        for (int i = 0; i < total_drones_; ++i)
        {
            if (i == drone_id_)
                continue;
            if (drone_positions_[i].allFinite() && distances2d_[i] > 1e-6)
                anchors.push_back({drone_positions_[i], distances2d_[i]});
        }

        if (anchors.size() < 3)
            return Eigen::Vector3d::Zero();

        double x0 = anchors[0].P.x();
        double y0 = anchors[0].P.y();
        double d0 = anchors[0].r;

        Eigen::MatrixXd A(anchors.size() - 1, 2);
        Eigen::VectorXd b(anchors.size() - 1);

        for (size_t i = 1; i < anchors.size(); ++i)
        {
            double xi = anchors[i].P.x();
            double yi = anchors[i].P.y();
            double di = anchors[i].r;

            A(i - 1, 0) = 2 * (xi - x0);
            A(i - 1, 1) = 2 * (yi - y0);
            b(i - 1) = (d0 * d0 - di * di) - (x0 * x0 - xi * xi) - (y0 * y0 - yi * yi);
        }

        Eigen::Vector2d xy = A.colPivHouseholderQr().solve(b);
        return Eigen::Vector3d(xy.x(), xy.y(), z_me);
    }

    // ----------------- 3D Trilateration -----------------
    Eigen::Vector3d trilateration3D()
    {
        struct Anchor
        {
            Eigen::Vector3d P;
            double r;
        };
        std::vector<Anchor> anchors;

        for (int i = 0; i < total_drones_; ++i)
        {
            if (i == drone_id_)
                continue;
            if (drone_positions_[i].allFinite() && distances3d_[i] > 1e-6)
                anchors.push_back({drone_positions_[i], distances3d_[i]});
        }

        if (anchors.size() < 4)
        {
            ROS_WARN("Need >=4 anchors for 3D trilateration, got %zu", anchors.size());
            return Eigen::Vector3d::Zero();
        }

        // 使用第一个点作为参考
        const Eigen::Vector3d p0 = anchors[0].P;
        const double r0p2 = anchors[0].r * anchors[0].r;

        int M = anchors.size() - 1;
        Eigen::MatrixXd A(M, 3);
        Eigen::VectorXd b(M);

        for (int i = 0; i < M; ++i)
        {
            Eigen::Vector3d pi = anchors[i + 1].P;
            double rip2 = anchors[i + 1].r * anchors[i + 1].r;

            A.row(i) = 2.0 * (pi - p0).transpose();
            b(i) = (rip2 - r0p2) - (pi.squaredNorm() - p0.squaredNorm());
        }

        Eigen::Vector3d xyz = A.colPivHouseholderQr().solve(b);

        // 添加调试信息
        ROS_INFO("3D Trilateration: %zu anchors, result: (%.3f, %.3f, %.3f)",
                 anchors.size(), xyz.x(), xyz.y(), xyz.z());

        return xyz;
    }

    // ----------------- 发布 -----------------
    void publishFusedPose(const Eigen::Vector3d &pos)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.w = 1.0;
        fused_odom_pub_.publish(msg);

        if (mode_ == "3d")
            ROS_INFO_THROTTLE(1.0, "Published 3D fused odom: (%.3f, %.3f, %.3f)", 
                pos.x(), pos.y(), pos.z());
        else
            ROS_INFO_THROTTLE(1.0, "Published 2D fused odom: (%.3f, %.3f, %.3f)", 
                pos.x(), pos.y(), pos.z());
    }
};

#endif
