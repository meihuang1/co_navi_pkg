#!/usr/bin/env python3
from flask import Flask, render_template, request, jsonify, url_for
import subprocess
import signal
import os
import threading
import rospy
from nav_msgs.msg import Odometry

STATIC_DIR = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/src/co_navi_pkg/static"
app = Flask(__name__, static_folder=STATIC_DIR)
current_process = None

speed_file = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/src/co_navi_pkg/control_ui/data/latest_speed.txt"


log_dir = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/log_files"
plot_script = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/src/co_navi_pkg/scripts/logs_plot.py"

# 路径映射
launch_paths = {
    "低速": {
        "视觉惯导定位算法": "F_low_imu_vision.launch",
        "卫星惯导定位算法": "F_low_imu_gps.launch",
        "视觉加高精度惯导定位算法": "F_low_imu_vision_high_precision.launch",
        "集群协同定位算法": "F_low_cooperative_navi.launch",
        "空地异构协同定位算法": "F_low_cooperative_navi_hetero.launch"
    },
    "中速": {
        "视觉惯导定位算法": "F_mid_imu_vision.launch",
        "卫星惯导定位算法": "F_mid_imu_gps.launch",
        "视觉加高精度惯导定位算法": "F_mid_imu_vision_high_precision.launch",
        "集群协同定位算法": "F_mid_cooperative_navi.launch"
    },
    "高速": {
        "视觉惯导定位算法": "F_high_imu_vision.launch",
        "卫星惯导定位算法": "F_high_imu_gps.launch",
        "视觉加高精度惯导定位算法": "F_high_imu_vision_high_precision.launch",
        "集群协同定位算法": "F_high_cooperative_navi.launch"
    }
}

base_path = "/home/adminn/ros_ws/Fast_Lab/ego-planner-swarm/src/co_navi_pkg/launch"
speed_dirs = {
    "低速": "low_speed/FinalTest",
    "中速": "mid_speed/FinalTest",
    "高速": "high_speed/FinalTest"
}

fusion_modes = {
    "视觉惯导定位算法": "imu_vision",
    "卫星惯导定位算法": "imu_gps",
    "视觉加高精度惯导定位算法": "imu_vision_high_precision",
    "集群协同定位算法": "cooperation_navigation",
    "空地异构协同定位算法": "cooperation_navigation_hetero",
}

@app.route("/current_speed")
def current_speed():
    if os.path.exists(speed_file):
        try:
            with open(speed_file, "r") as f:
                speed = float(f.read().strip())
        except:
            speed = 0.0
    else:
        speed = 0.0
    return jsonify({"speed": round(speed, 2)})

@app.route("/")
def index():
    return render_template(
        "index.html",
        speeds=list(launch_paths.keys()),
        algos=list(launch_paths["低速"].keys()),
        launch_paths=launch_paths  # 必须传给模板
    )

@app.route("/load", methods=["POST"])
def load():
    global current_process
    speed = request.json["speed"]
    algo = request.json["algo"]

    if current_process:
        return jsonify({"status": "error", "message": "已有任务在运行，请先停止"})

    launch_file = os.path.join(base_path, speed_dirs[speed], launch_paths[speed][algo])
    cmd = ["roslaunch", launch_file]

    current_process = subprocess.Popen(cmd, preexec_fn=os.setsid,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return jsonify({"status": "ok", "message": f"已加载 {speed} + {algo}"})

@app.route("/run", methods=["POST"])
def run():
    cmd = ["rostopic", "pub", "/run_cmd", "std_msgs/Int32", "data: 1", "-1"]
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return jsonify({"status": "ok", "message": "已发布运行指令"})

@app.route("/stop", methods=["POST"])
def stop():
    global current_process
    if current_process:
        os.killpg(os.getpgid(current_process.pid), signal.SIGINT)
        current_process = None
        return jsonify({"status": "ok", "message": "已停止所有进程"})
    return jsonify({"status": "error", "message": "当前没有运行任务"})

@app.route("/analyze", methods=["POST"])
def analyze():
    speed = request.json["speed"]
    algo = request.json["algo"]

    run_mode = {"低速": "low", "中速": "mid", "高速": "high"}[speed]
    fusion_mode = fusion_modes[algo]

    log_file = f"{run_mode}_{fusion_mode}.csv"
    log_path = os.path.join(log_dir, log_file)

    if not os.path.exists(log_path):
        return jsonify({"status": "error", "message": f"日志文件不存在: {log_file}"})

    try:
        result = subprocess.run(
            ["python3", plot_script, log_path],
            capture_output=True, text=True, check=True
        )
        output = result.stdout.strip().splitlines()

        stats = []
        images = []
        for line in output:
            if line.startswith("PLOT_IMAGE:"):
                images.append(os.path.basename(line.split(":", 1)[1].strip()))
            elif line.startswith("STATS:") or line.strip() == "":
                continue
            else:
                stats.append(line)

        return jsonify({
            "status": "ok",
            "stats": "\n".join(stats),
            "images": [url_for("static", filename=img) for img in images]
        })
    except subprocess.CalledProcessError as e:
        return jsonify({"status": "error", "message": f"分析失败: {e.stderr}"})

if __name__ == "__main__":
    # Flask debug 关闭，避免 reloader 影响 ROS 节点
    app.run(host="127.0.0.1", port=6677, debug=False)
