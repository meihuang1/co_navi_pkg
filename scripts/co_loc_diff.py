#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from collections import deque
import tf.transformations as tf_trans

class CooperativePoseDifferentiator:
    def __init__(self):
        rospy.init_node('cooperative_pose_differentiator')
        
        # 参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.diff_rate = rospy.get_param('~diff_rate', 20.0)  # 差分频率，默认20Hz
        
        # 构建话题名称
        drone_prefix = f"/drone_{self.drone_id}"
        print(f"drone_prefix: {drone_prefix}")
        # 订阅 cooperative pose
        self.cooperative_pose_sub = rospy.Subscriber(
            f'{drone_prefix}/odom_fused_cooperative_pose', 
            Odometry, 
            self.cooperative_pose_callback
        )
        
        # 发布差分后的速度
        self.velocity_pub = rospy.Publisher(
            f'{drone_prefix}/cooperative_velocity_pose_diff', 
            Odometry, 
            queue_size=10
        )
        
        # 发布差分后的位置和速度（Odometry格式）
        self.odom_pub = rospy.Publisher(
            f'{drone_prefix}/cooperative_odom_diff', 
            Odometry, 
            queue_size=10
        )
        
        # 数据缓存
        self.max_history = 50  # 最大历史记录数
        self.pos_history = deque(maxlen=self.max_history)
        self.time_history = deque(maxlen=self.max_history)
        
        # 差分参数
        self.last_published_time = rospy.Time(0)
        self.publish_interval = 1.0 / self.diff_rate  # 发布间隔
        
        # 速度平滑参数
        self.vel_filter_alpha = 0.3  # 速度滤波系数
        self.filtered_velocity = np.zeros(3)
        
        rospy.loginfo(f"Cooperative Pose Differentiator initialized for drone_{self.drone_id}")
        rospy.loginfo(f"Differentiation rate: {self.diff_rate} Hz")
        
    def cooperative_pose_callback(self, msg):
        """处理cooperative pose消息"""
        timestamp = msg.header.stamp
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 存储位置和时间
        self.pos_history.append(position)
        self.time_history.append(timestamp)
        
        # 检查是否需要发布（控制频率）
        current_time = rospy.Time.now()
        if (current_time - self.last_published_time).to_sec() >= self.publish_interval:
            self.compute_and_publish_derivative()
            self.last_published_time = current_time
            
        rospy.loginfo("Got cooperative pose msg")
        current_time = rospy.Time.now()
        rospy.loginfo(f"now={current_time.to_sec():.3f}, last={self.last_published_time.to_sec():.3f}, interval={self.publish_interval:.3f}")
        if (current_time - self.last_published_time).to_sec() >= self.publish_interval:
            rospy.loginfo("Calling derivative...")
            self.compute_and_publish_derivative()
            self.last_published_time = current_time
        
        
    def compute_and_publish_derivative(self):
        """计算位置导数并发布"""
        if len(self.pos_history) < 2 or len(self.time_history) < 2:
            return
        
        # 获取最新的两个位置和时间
        pos_current = self.pos_history[-1]
        pos_previous = self.pos_history[-2]
        time_current = self.time_history[-1]
        time_previous = self.time_history[-2]
        
        # 计算时间差
        dt = (time_current - time_previous).to_sec()
        
        if dt > 0.001 and dt < 1.0:  # 避免异常时间间隔
            # 计算原始速度
            velocity_raw = (pos_current - pos_previous) / dt
            
            # 应用低通滤波平滑速度
            self.filtered_velocity = (
                self.vel_filter_alpha * velocity_raw + 
                (1 - self.vel_filter_alpha) * self.filtered_velocity
            )
            
            # 发布Float64MultiArray格式的速度
            self.publish_velocity_array(pos_current, self.filtered_velocity)
            
            # 发布Odometry格式的完整信息
            self.publish_odometry(pos_current, self.filtered_velocity, time_current)
            
            rospy.loginfo(f"Position: [{pos_current[0]:.3f}, {pos_current[1]:.3f}, {pos_current[2]:.3f}]")
            rospy.loginfo(f"Velocity: [{self.filtered_velocity[0]:.3f}, {self.filtered_velocity[1]:.3f}, {self.filtered_velocity[2]:.3f}]")
            rospy.loginfo(f"dt: {dt:.4f}s")
            rospy.loginfo("---")
    
    def publish_velocity_array(self, position, velocity):
        """发布Float64MultiArray格式的速度"""
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.child_frame_id = f"drone_{self.drone_id}/base_link"
        
        # 位置
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        
        # 速度
        msg.twist.twist.linear.x = velocity[0]
        msg.twist.twist.linear.y = velocity[1]
        msg.twist.twist.linear.z = velocity[2]
        
        # 协方差（简单设置）
        msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                              0, 0.01, 0, 0, 0, 0,
                              0, 0, 0.01, 0, 0, 0,
                              0, 0, 0, 0.01, 0, 0,
                              0, 0, 0, 0, 0.01, 0,
                              0, 0, 0, 0, 0, 0.01]
        
        msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0,
                               0, 0, 0, 0, 0.1, 0,
                               0, 0, 0, 0, 0, 0.1]
        
        self.velocity_pub.publish(msg)
    
    def publish_odometry(self, position, velocity, timestamp):
        """发布Odometry格式的完整信息"""
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = "world"
        msg.child_frame_id = f"drone_{self.drone_id}/base_link"
        
        # 位置
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        msg.pose.pose.orientation.w = 1.0
        
        # 速度
        msg.twist.twist.linear.x = velocity[0]
        msg.twist.twist.linear.y = velocity[1]
        msg.twist.twist.linear.z = velocity[2]
        
        # 协方差（简单设置）
        msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                              0, 0.01, 0, 0, 0, 0,
                              0, 0, 0.01, 0, 0, 0,
                              0, 0, 0, 0.01, 0, 0,
                              0, 0, 0, 0, 0.01, 0,
                              0, 0, 0, 0, 0, 0.01]
        
        msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0,
                               0, 0, 0, 0, 0.1, 0,
                               0, 0, 0, 0, 0, 0.1]
        
        self.odom_pub.publish(msg)
    
    def run(self):
        """主运行函数"""
        rospy.loginfo("Starting Cooperative Pose Differentiator...")
        rospy.spin()

if __name__ == "__main__":
    try:
        differentiator = CooperativePoseDifferentiator()
        differentiator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")