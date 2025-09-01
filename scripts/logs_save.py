#!/usr/bin/env python3
import rospy
import os
import csv
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

class TopicLogger:
    def __init__(self):
        # 日志保存目录
        self.log_dir = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/log_files"
        os.makedirs(self.log_dir, exist_ok=True)

        # 从 param 读取 run_mode 和 fusion_mode
        self.run_mode = rospy.get_param("~run_mode", "default")
        self.fusion_mode = rospy.get_param("~fusion_mode", "imu_gps")

        # 输出文件名：runmode + fusionmode
        file_path = os.path.join(
            self.log_dir, f"{self.run_mode}_{self.fusion_mode}.csv"
        )
        self.file = open(file_path, "w", newline="")
        self.writer = csv.writer(self.file)

        # 写表头（groundtruth + fusion）
        self.writer.writerow([
            "stamp_gt", "x", "y", "z", "vx", "vy", "vz",
            "stamp_fusion", "fusion_data[]"
        ])

        # 设置订阅
        topics_float = {
            "imu_gps": "/drone_0/error_ig_global",
            "imu_vision": "/drone_0/error_iv",
            "imu_vision_high_precision": "/drone_0/error_iv_hp",
            "cooperation_navigation": "/drone_0/co_loc_error",
            "cooperation_navigation_hetero": "/drone_0/co_loc_error",
        }

        self.fusion_topic = topics_float.get(self.fusion_mode, None)

        if self.fusion_topic:
            rospy.Subscriber(self.fusion_topic, Float64MultiArray, self.fusion_callback)
        rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, self.odom_callback)

        rospy.loginfo(
            "TopicLogger initialized, logging to %s (mode=%s, fusion=%s)",
            file_path,
            self.run_mode,
            self.fusion_mode,
        )

    def fusion_callback(self, msg: Float64MultiArray):
        stamp = rospy.Time.now().to_sec()
        # ground truth 部分留空，只写 fusion
        row = ["", "", "", "", "", "", "", stamp] + list(msg.data)
        self.writer.writerow(row)
        self.file.flush()

    def odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp.to_sec()
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        # fusion 部分留空，只写 odom
        row = [stamp, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, "", ""]
        self.writer.writerow(row)
        self.file.flush()

    def __del__(self):
        self.file.close()


if __name__ == "__main__":
    rospy.init_node("topic_logger")
    logger = TopicLogger()
    rospy.spin()
