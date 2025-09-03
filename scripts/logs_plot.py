#!/usr/bin/env python3
import csv
import math
import sys
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

STATIC_DIR = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/src/co_navi_pkg/static"
os.makedirs(STATIC_DIR, exist_ok=True)

def load_csv(file_path):
    times_gt, pos_gt, vel_gt = [], [], []
    times_fused, pos_fused, vel_fused = [], [], []
    times_err, pos_err, vel_err = [], [], []

    with open(file_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            # ground truth
            if row[0]:
                t = float(row[0])
                times_gt.append(t)
                pos_gt.append([float(row[1]), float(row[2]), float(row[3])])
                vel_gt.append([float(row[4]), float(row[5]), float(row[6])])
            # fused odom
            if row[10]:
                t = float(row[10])
                times_fused.append(t)
                pos_fused.append([float(row[11]), float(row[12]), float(row[13])])
                vel_fused.append([float(row[14]), float(row[15]), float(row[16])])
            # fusion err_norm
            if row[7]:
                t = float(row[7])
                times_err.append(t)
                pos_err.append(float(row[8]))
                vel_err.append(float(row[9]))

    # 时间对齐
    if times_gt:
        t0 = times_gt[0]
        times_gt = [t - t0 for t in times_gt]
    if times_fused:
        t0 = times_fused[0]
        times_fused = [t - t0 for t in times_fused]
    if times_err:
        t0 = times_err[0]
        times_err = [t - t0 for t in times_err]

    return times_gt, pos_gt, vel_gt, times_fused, pos_fused, vel_fused, times_err, pos_err, vel_err

def compute_stats(values, label=""):
    if not values:
        return
    n = len(values)
    mean_val = sum(values)/n
    max_val = max(values)
    mse = sum(v*v for v in values)/n
    print(f"{label}: mean={mean_val:.4f}, max={max_val:.4f}, mse={mse:.4f}")
    

def vector_norm_list(vec_list):
    return [math.sqrt(x[0]**2 + x[1]**2 + x[2]**2) for x in vec_list]

def compute_stats_table(pos_err, vel_err):
    # 计算指标
    def stats(values):
        n = len(values)
        mean_val = sum(values)/n
        max_val = max(values)
        mse = sum(v*v for v in values)/n
        return mean_val, max_val, mse

    pos_mean, pos_max, pos_mse = stats(pos_err) if pos_err else (0,0,0)
    vel_mean, vel_max, vel_mse = stats(vel_err) if vel_err else (0,0,0)

    # 构建表格
    table = f"""
+---------------------+----------+---------+---------+
|       Metric        |   Mean   |   Max   |   MSE   |
+---------------------+----------+---------+---------+
| Position error [m]  | {pos_mean:7.4f}  | {pos_max:7.4f} | {pos_mse:7.4f} |
| Velocity error [m/s]| {vel_mean:7.4f}  | {vel_max:7.4f} | {vel_mse:7.4f} |
+---------------------+----------+---------+---------+
"""
    print("STATS_OUTPUT:")
    print(table)


def plot_and_save(file_path):
    base_name = os.path.basename(file_path).replace(".csv","")
    times_gt, pos_gt, vel_gt, times_fused, pos_fused, vel_fused, times_err, pos_err, vel_err = load_csv(file_path)

    # 计算统计数据
    # compute_stats(pos_err, "Position error norm")
    # compute_stats(vel_err, "Velocity error norm")
    compute_stats_table(pos_err, vel_err)
    fig, axs = plt.subplots(2,2, figsize=(12,8))

    # subplot (0,0) 位置对比
    axs[0,0].plot(times_gt, vector_norm_list(pos_gt), label="GT pos")
    axs[0,0].plot(times_fused, vector_norm_list(pos_fused), label="Fused pos")
    axs[0,0].set_ylabel("Position [m]")
    axs[0,0].legend()
    axs[0,0].set_title("Position comparison")

    # subplot (0,1) 速度对比
    axs[0,1].plot(times_gt, vector_norm_list(vel_gt), label="GT vel")
    axs[0,1].plot(times_fused, vector_norm_list(vel_fused), label="Fused vel")
    axs[0,1].set_ylabel("Velocity [m/s]")
    axs[0,1].legend()
    axs[0,1].set_title("Velocity comparison")

    # subplot (1,0) 位置误差
    axs[1,0].plot(times_err, pos_err, label="Position error norm", color="b")
    axs[1,0].set_ylabel("Position error [m]")
    axs[1,0].legend()
    axs[1,0].set_xlabel("Time [s]")

    # subplot (1,1) 速度误差
    axs[1,1].plot(times_err, vel_err, label="Velocity error norm", color="r")
    axs[1,1].set_ylabel("Velocity error [m/s]")
    axs[1,1].legend()
    axs[1,1].set_xlabel("Time [s]")

    plt.suptitle(base_name, fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.95])

    save_path = os.path.join(STATIC_DIR, f"{base_name}_error.png")
    plt.savefig(save_path, dpi=300)
    plt.close(fig)

    # 打印图片路径给 Flask
    print("PLOT_IMAGE:", save_path)

if __name__ == "__main__":
    if len(sys.argv)<2:
        print("Usage: logs_plot.py <logfile.csv>")
        sys.exit(1)

    file_path = sys.argv[1]
    plot_and_save(file_path)