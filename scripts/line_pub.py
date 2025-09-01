#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import numpy as np

class SmoothLineTrajectory:
    def __init__(self):
        # ==== 从 ROS 参数读取 ====
        self.drone_id  = rospy.get_param("~drone_id", 0)
        self.v_final   = rospy.get_param("~v_final", 40.0)
        self.a_max     = rospy.get_param("~a_max", 5.0)
        self.x_init    = rospy.get_param("~init_x", 0.0)
        self.y_init    = rospy.get_param("~init_y", 0.0)
        self.x_target  = rospy.get_param("~x_target", 200.0)
        self.y_target  = rospy.get_param("~y_target", 0.0)
        self.z_height  = rospy.get_param("~init_z", 5.0)
        
        self.rate_hz   = 100.0

        self.dt = 1.0 / self.rate_hz
        self.t = 0.0

        # ==== 状态量 ====
        self.pos_x = self.x_init
        self.pos_y = self.y_init
        self.vel   = 0.0
        self.acc   = 0.0
        self.running = False

        # ==== 预计算轨迹参数 ====
        self.prepare_profile()

        # ==== ROS pub/sub ====
        topic_name = f"/drone_{self.drone_id}_visual_slam/odom"
        self.pub = rospy.Publisher(topic_name, Odometry, queue_size=10)
        rospy.Subscriber('/run_cd', Int32, self.run_cmd_callback)

        rospy.loginfo(f"[Trajectory Node] drone_id={self.drone_id}, publishing on {topic_name}")

    def prepare_profile(self):
        """ 根据 v_final, a_max, x_target-x_init 计算轨迹参数 """
        v_max = self.v_final
        a_max = self.a_max
        dist = np.hypot(self.x_target - self.x_init, self.y_target - self.y_init)

        # 最小需求距离（加速到v_max再减速到0）
        s_need = 2.0 * (v_max**2) / a_max
        if dist >= s_need:
            self.v_peak = v_max
        else:
            self.v_peak = np.sqrt(0.5 * a_max * dist)

        self.T_ramp = 2.0 * self.v_peak / a_max
        self.s_ramp = self.v_peak**2 / a_max
        if dist >= 2 * self.s_ramp:
            self.s_cruise = dist - 2 * self.s_ramp
            self.T_cruise = self.s_cruise / self.v_peak
        else:
            self.s_cruise = 0.0
            self.T_cruise = 0.0

        self.T_total = 2 * self.T_ramp + self.T_cruise

        rospy.loginfo(f"[Trajectory] v_peak={self.v_peak:.3f}, T_ramp={self.T_ramp:.3f}, "
                      f"T_cruise={self.T_cruise:.3f}, T_total={self.T_total:.3f}")

    def run_cmd_callback(self, msg):
        self.running = (msg.data == 1)
        if self.running:
            self.t = 0.0
            self.pos_x = self.x_init
            self.pos_y = self.y_init
            self.vel = 0.0
            self.acc = 0.0

    def compute_state(self):
        """ 根据 t 计算 pos, vel, acc """
        T_ramp = self.T_ramp
        T_cruise = self.T_cruise
        T_total = self.T_total
        a_max = self.a_max
        dist = np.hypot(self.x_target - self.x_init, self.y_target - self.y_init)

        ti = self.t

        if ti < T_ramp:
            tau = ti
            self.acc = a_max * (np.sin(np.pi * tau / T_ramp))**2
        elif ti < T_ramp + T_cruise:
            self.acc = 0.0
        elif ti < T_total:
            tau = ti - (T_ramp + T_cruise)
            self.acc = - a_max * (np.sin(np.pi * tau / T_ramp))**2
        else:
            self.acc = 0.0

        # 积分更新速度和位置
        self.vel += self.acc * self.dt
        if self.vel < 0:
            self.vel = 0.0

        # 按比例更新 x, y 位置
        ratio = (self.vel * self.dt) / dist if dist > 0 else 0.0
        self.pos_x += (self.x_target - self.x_init) * ratio
        self.pos_y += (self.y_target - self.y_init) * ratio

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"

        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.position.z = self.z_height

        odom.twist.twist.linear.x = self.vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        # twist.covariance 用作输出加速度
        odom.twist.covariance[0] = self.acc

        self.pub.publish(odom)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.running:
                self.compute_state()
                self.t += self.dt
            self.publish_odom()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("smooth_line_trajectory")
    traj = SmoothLineTrajectory()
    traj.run()
