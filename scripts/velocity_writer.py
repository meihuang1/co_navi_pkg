#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import math
import time
import os

speed_file = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/src/co_navi_pkg/control_ui/data/latest_speed.txt"

def callback(msg):
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    vz = msg.twist.twist.linear.z
    speed = math.sqrt(vx**2 + vy**2 + vz**2)
    with open(speed_file, "w") as f:
        f.write(f"{speed:.2f}")

if __name__ == "__main__":
    rospy.init_node('odom_writer', anonymous=True)
    rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, callback)
    rospy.spin()
