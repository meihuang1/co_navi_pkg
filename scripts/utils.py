import numpy as np

# 常量
GRAVITY_Z = 9.80665  # m/s^2

def quat_mult(a, b):
    """四元数乘法: a*b"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_to_ang_vel(q_prev, q_curr, dt):
    """由前后两个四元数求角速度（rad/s）"""
    q_prev_conj = np.array([q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]])
    dq = quat_mult(q_prev_conj, q_curr)
    w = np.clip(dq[0], -1.0, 1.0)
    v = dq[1:]
    v_norm = np.linalg.norm(v)
    phi = 2.0 * np.arctan2(v_norm, w)
    if v_norm < 1e-8:
        rot_vec = 2.0 * v
    else:
        axis = v / v_norm
        rot_vec = axis * phi
    return rot_vec / dt

def integrate_lin_acc(prev_vel, lin_acc, prev_lin_acc, dt):
    """梯形积分加速度 -> 速度"""
    return prev_vel + 0.5 * (prev_lin_acc + lin_acc) * dt

def integrate_vel(prev_pos, vel, dt):
    """梯形积分速度 -> 位置"""
    return prev_pos + vel * dt

def integrate_ang_acc(prev_ang_vel, ang_acc, prev_ang_acc, dt):
    """梯形积分角加速度 -> 角速度"""
    return prev_ang_vel + 0.5 * (prev_ang_acc + ang_acc) * dt

def integrate_ang_vel(prev_quat, ang_vel, dt):
    """角速度积分 -> 新四元数姿态"""
    omega_mag = np.linalg.norm(ang_vel)
    if omega_mag * dt < 1e-8:
        dq = np.array([1.0,
                       0.5*ang_vel[0]*dt,
                       0.5*ang_vel[1]*dt,
                       0.5*ang_vel[2]*dt])
    else:
        theta = omega_mag * dt
        axis = ang_vel / omega_mag
        dq = np.array([np.cos(theta/2.0),
                       *(axis * np.sin(theta/2.0))])
    quat_new = quat_mult(prev_quat, dq)
    return quat_new / np.linalg.norm(quat_new)