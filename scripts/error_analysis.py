#! /usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from collections import deque


class ErrorAnalyzer:
    def __init__(self):
        rospy.init_node('error_analyzer', anonymous=True)

        # 参数
        self.drone_id = rospy.get_param('~drone_id', 0)
        drone_prefix = f"/drone_{self.drone_id}"

        # ---------------- 数据缓存 ----------------
        self.max_data_points = 1000
        self.truth_data = deque(maxlen=self.max_data_points)

        
        self.ig_global_vel = None
        # ---------------- 订阅话题 ----------------
        # ground truth
        self.truth_sub = rospy.Subscriber(
            f'{drone_prefix}_visual_slam/odom', Odometry, self.truth_callback
        )

        # 三种融合算法 Odom
        self.ig_global_sub = rospy.Subscriber(
            f'{drone_prefix}/odom_fused_IG_global', Odometry, self.ig_global_callback
        )
        self.igv_sub = rospy.Subscriber(
            f'{drone_prefix}/odom_fused_IGV', Odometry, self.igv_callback
        )
        self.iv_sub = rospy.Subscriber(
            f'{drone_prefix}/odom_fused_IV', Odometry, self.iv_callback
        )   
        
        # cooperative pose (仅 drone_0 用)
        if self.drone_id == 0:
            rospy.Subscriber(
                f'{drone_prefix}/odom_fused_cooperative_pose',
                Odometry,
                self.cooperative_pose_callback
            )
            self.co_loc_error_pub = rospy.Publisher(
                f'{drone_prefix}/co_loc_error',
                Float64MultiArray,
                queue_size=10
            )

        # ---------------- 发布话题 ----------------
        self.ig_global_error_pub = rospy.Publisher(
            f'{drone_prefix}/error_ig_global', Float64MultiArray, queue_size=10
        )
        self.igv_error_pub = rospy.Publisher(
            f'{drone_prefix}/error_igv', Float64MultiArray, queue_size=10
        )
        self.iv_error_pub = rospy.Publisher(
            f'{drone_prefix}/error_iv', Float64MultiArray, queue_size=10
        )

        rospy.loginfo(f"Error Analyzer initialized for drone_{self.drone_id}")

    # ---------------- 工具函数 ----------------
    def compute_and_publish_error(self, est_pos, est_vel, pub):
        """
        输入：估计位置/速度 (np.array)，truth，发布误差
        输出：Float64MultiArray [ex,ey,ez, evx,evy,evz, pos_norm, vel_norm]
        """
        if len(self.truth_data) == 0:
            return

        truth_pos, truth_vel = self.truth_data[-1]

        e_pos = est_pos - truth_pos
        e_vel = est_vel - truth_vel
        pos_norm = np.linalg.norm(e_pos)
        vel_norm = np.linalg.norm(e_vel)

        msg_out = Float64MultiArray()
        msg_out.data = [
            e_pos[0], e_pos[1], e_pos[2],
            pos_norm,
            e_vel[0], e_vel[1], e_vel[2],
            vel_norm
        ]
        pub.publish(msg_out)

    # ---------------- 回调函数 ----------------
    def truth_callback(self, msg):
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.truth_data.append((pos, vel))

    def ig_global_callback(self, msg: Odometry):
        est_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        est_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.ig_global_vel = est_vel
        self.compute_and_publish_error(est_pos, est_vel, self.ig_global_error_pub)

    def igv_callback(self, msg: Odometry):
        est_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        est_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.compute_and_publish_error(est_pos, est_vel, self.igv_error_pub)

    def iv_callback(self, msg: Odometry):
        est_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        est_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        self.compute_and_publish_error(est_pos, est_vel, self.iv_error_pub)

    def cooperative_pose_callback(self, msg: Odometry):
        est_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        est_vel = np.zeros(3)
         # est_vel = np.array([
        #     msg.twist.twist.linear.x,
        #     msg.twist.twist.linear.y,
        #     msg.twist.twist.linear.z
        # ])
        est_vel = self.ig_global_vel
        self.compute_and_publish_error(est_pos, est_vel, self.co_loc_error_pub)

    # ---------------- 主循环 ----------------
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        analyzer = ErrorAnalyzer()
        analyzer.run()
    except rospy.ROSInterruptException:
        pass
