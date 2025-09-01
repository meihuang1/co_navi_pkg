#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        # ==== ROS 参数 ====
        self.drone_id = rospy.get_param("~drone_id", 0)

        # ==== 话题名 ====
        odom_topic = f"/drone_{self.drone_id}_visual_slam/odom"
        path_topic = f"/drone_{self.drone_id}_odom_visualization/path"

        # ==== 发布和订阅 ====
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "world"

        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        rospy.loginfo(f"[OdomToPath] Subscribing {odom_topic}, Publishing {path_topic}")

    def odom_callback(self, msg: Odometry):
        # 构造 PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "world"
        pose.pose = msg.pose.pose

        # 累积轨迹
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose)

        # 发布 path
        self.path_pub.publish(self.path)

if __name__ == "__main__":
    rospy.init_node("odom_to_path")
    node = OdomToPath()
    rospy.spin()
