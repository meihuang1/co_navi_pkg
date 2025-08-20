#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class DistanceCalculator:
    def __init__(self):
        rospy.init_node('distance_calculator', anonymous=True)
        
        # 获取参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        self.total_drones = rospy.get_param('~total_drones', 4)
        self.noise_std_2d = rospy.get_param('~noise_std_2d', 0.02)  # 2D距离噪声标准差 (米)
        self.noise_std_3d = rospy.get_param('~noise_std_3d', 0.02)  # 3D距离噪声标准差 (米)
        
        # 存储所有无人机位置（包括当前无人机）
        self.drone_positions = {}
        
        # 订阅所有无人机的里程计（包括当前无人机）
        self.odom_subs = {}
        for i in range(self.total_drones):
            topic = f'/drone_{i}_visual_slam/odom'
            self.odom_subs[i] = rospy.Subscriber(
                topic, Odometry, 
                lambda msg, drone_id=i: self.odomCallback(msg, drone_id)
            )
        
        # 发布2D和3D距离话题
        self.dist2d_pubs = {}
        self.dist3d_pubs = {}
        for i in range(self.total_drones):
            if i != self.drone_id:
                # 2D距离话题
                topic_2d = f'/dist2d_{self.drone_id}_{i}'
                self.dist2d_pubs[i] = rospy.Publisher(topic_2d, Float64, queue_size=1)
                
                # 3D距离话题
                topic_3d = f'/dist3d_{self.drone_id}_{i}'
                self.dist3d_pubs[i] = rospy.Publisher(topic_3d, Float64, queue_size=1)
        
        # 修复：使用正确的Python语法
        self.timer = rospy.Timer(rospy.Duration(0.1), self.calculateDistances)  # 10Hz
        
        rospy.loginfo(f"Distance Calculator initialized for drone_{self.drone_id}")
        rospy.loginfo(f"Publishing 2D and 3D distances with noise: 2D={self.noise_std_2d}m, 3D={self.noise_std_3d}m")
    
    def odomCallback(self, msg, drone_id):
        """接收无人机里程计数据"""
        self.drone_positions[drone_id] = Point(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # 调试信息
        # if drone_id == self.drone_id:
        #     rospy.loginfo(f"Current drone_{self.drone_id} position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})")
        # else:
        #     rospy.loginfo(f"Received drone_{drone_id} position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})")
    
    def calculateDistances(self, event):
        """计算并发布到其他无人机的2D和3D距离"""
        # 检查是否所有无人机位置都已接收
        if len(self.drone_positions) < self.total_drones:
            # rospy.logwarn(f"Waiting for drone positions: {len(self.drone_positions)}/{self.total_drones}")
            return
        
        # 检查当前无人机位置是否有效
        if self.drone_id not in self.drone_positions:
            rospy.logwarn(f"Current drone_{self.drone_id} position not available")
            return
        
        current_pos = self.drone_positions[self.drone_id]
        
        # 计算并发布到其他所有无人机的距离
        for other_drone_id in range(self.total_drones):
            if other_drone_id == self.drone_id:
                continue  # 跳过自己
                
            if other_drone_id not in self.drone_positions:
                rospy.logwarn(f"Drone_{other_drone_id} position not available")
                continue
            
            other_pos = self.drone_positions[other_drone_id]
            
            # 计算2D距离（忽略Z轴）
            true_distance_2d = self.calculateDistance2D(current_pos, other_pos)
            
            # 计算3D距离（包含Z轴）
            true_distance_3d = self.calculateDistance3D(current_pos, other_pos)
            
            # 添加高斯噪声
            noisy_distance_2d = true_distance_2d + np.random.normal(0, self.noise_std_2d)
            noisy_distance_3d = true_distance_3d + np.random.normal(0, self.noise_std_3d)
            
            # 发布2D距离
            dist2d_msg = Float64()
            dist2d_msg.data = noisy_distance_2d
            self.dist2d_pubs[other_drone_id].publish(dist2d_msg)
            
            # 发布3D距离
            dist3d_msg = Float64()
            dist3d_msg.data = noisy_distance_3d
            self.dist3d_pubs[other_drone_id].publish(dist3d_msg)
            
            rospy.loginfo(f"drone_{self.drone_id} to drone_{other_drone_id}: "
                         f"2D={noisy_distance_2d:.2f}m (true: {true_distance_2d:.2f}m), "
                         f"3D={noisy_distance_3d:.2f}m (true: {true_distance_3d:.2f}m)")
    
    def calculateDistance2D(self, pos1, pos2):
        """计算2D距离（忽略Z轴）"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return np.sqrt(dx*dx + dy*dy)
    
    def calculateDistance3D(self, pos1, pos2):
        """计算3D距离（包含Z轴）"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return np.sqrt(dx*dx + dy*dy + dz*dz)

if __name__ == '__main__':
    try:
        DistanceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass