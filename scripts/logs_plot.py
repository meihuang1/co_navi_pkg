#!/usr/bin/env python3
import os
import sys
import csv
import math
import matplotlib.pyplot as plt

def read_error_file(error_file):
    stamps, pos_norm, vel_norm = [], [], []
    with open(error_file, "r") as f:
        reader = csv.reader(f)
        next(reader)  # 跳过表头
        for row in reader:
            if not row: 
                continue
            stamps.append(float(row[0]))
            pos_norm.append(float(row[4]))  # pos_norm
            vel_norm.append(float(row[8]))  # vel_norm
    return stamps, pos_norm, vel_norm

def read_odom_file(odom_file):
    stamps, vel_norm = [], []
    with open(odom_file, "r") as f:
        reader = csv.reader(f)
        next(reader)  # 跳过表头
        for row in reader:
            if not row: 
                continue
            stamps.append(float(row[0]))
            vx, vy, vz = float(row[4]), float(row[5]), float(row[6])
            vel_norm.append(math.sqrt(vx*vx + vy*vy + vz*vz))
    return stamps, vel_norm

def compute_stats(arr):
    n = len(arr)
    if n == 0:
        return {"max": 0, "mean": 0, "rms": 0}
    max_val = max(arr)
    mean_val = sum(arr) / n
    rms_val = math.sqrt(sum(x*x for x in arr) / n)
    return {"max": max_val, "mean": mean_val, "rms": rms_val}

def analyze_logs(error_file, odom_file):
    # 读文件
    t_error, pos_norm, vel_norm = read_error_file(error_file)
    t_odom, odom_vel_norm = read_odom_file(odom_file)

    # 统计
    pos_stats = compute_stats(pos_norm)
    vel_stats = compute_stats(vel_norm)

    print("位置误差 pos_norm:", pos_stats)
    print("速度误差 vel_norm:", vel_stats)

    # 文件名决定标题
    run_mode = os.path.basename(error_file).replace(".csv", "")

    # 画图
    plt.figure()
    plt.plot(t_error, pos_norm, label="pos error norm")
    plt.xlabel("time [s]")
    plt.ylabel("position error [m]")
    plt.title(f"Position Error ({run_mode})")
    plt.grid()
    plt.legend()
    plt.savefig(run_mode + "_pos_error.png", dpi=300)

    plt.figure()
    plt.plot(t_error, vel_norm, label="vel error norm")
    plt.xlabel("time [s]")
    plt.ylabel("velocity error [m/s]")
    plt.title(f"Velocity Error ({run_mode})")
    plt.grid()
    plt.legend()
    plt.savefig(run_mode + "_vel_error.png", dpi=300)

    plt.figure()
    plt.plot(t_odom, odom_vel_norm, label="ground truth speed")
    plt.xlabel("time [s]")
    plt.ylabel("velocity norm [m/s]")
    plt.title(f"Ground Truth Velocity ({run_mode})")
    plt.grid()
    plt.legend()
    plt.savefig(run_mode + "_truth_vel.png", dpi=300)

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python analyze_logs.py error_file.csv odom_file.csv")
        sys.exit(1)

    error_file = sys.argv[1]
    odom_file = sys.argv[2]
    analyze_logs(error_file, odom_file)
