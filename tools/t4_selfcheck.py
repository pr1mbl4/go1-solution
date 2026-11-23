#!/usr/bin/env python3
"""T4 selfcheck: RPE (Relative Pose Error) для /odometry/filtered.

Сравнивает относительное движение между /odometry/filtered и
/odom/ground_truth на интервалах 0.5, 1.0, 2.0 сек.
Выводит среднюю ошибку трансляции (м) и вращения (град).
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy),
                      1 - 2 * (qy * qy + qz * qz))


def pose_to_matrix(x, y, yaw):
    """2D pose to 3x3 matrix."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1],
    ])


def inv_matrix(T):
    """Inverse of 3x3 matrix for 2D pose."""
    R = T[:2, :2]
    t = T[:2, 2]
    Ti = np.eye(3)
    Ti[:2, :2] = R.T
    Ti[:2, 2] = -R.T @ t
    return Ti


def pose_from_msg(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w)


class T4Selfcheck(Node):
    def __init__(self):
        super().__init__('t4_selfcheck')
        self.filtered = None
        self.ground_truth = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_filtered, qos)
        self.create_subscription(
            Odometry, '/odom/ground_truth', self.on_gt, qos)

    def on_filtered(self, msg):
        self.filtered = msg

    def on_gt(self, msg):
        self.ground_truth = msg

    def collect_trajectory(self, vx=0.25, duration=10.0, hz=20.0):
        traj_P = []
        traj_Q = []
        cmd = Twist()
        cmd.linear.x = vx

        dt = 1.0 / hz
        n_steps = int(duration * hz)
        for _ in range(n_steps):
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=dt)
            if self.filtered is not None and self.ground_truth is not None:
                P = pose_from_msg(self.filtered)
                Q = pose_from_msg(self.ground_truth)
                traj_P.append(pose_to_matrix(*P))
                traj_Q.append(pose_to_matrix(*Q))

        cmd.linear.x = 0.0
        for _ in range(10):
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        return traj_P, traj_Q

    def compute_rpe(self, traj_P, traj_Q, delta):
        errs_trans = []
        errs_rot = []
        n = len(traj_P)
        if n <= delta:
            return None

        for i in range(n - delta):
            j = i + delta
            dQ = inv_matrix(traj_Q[i]) @ traj_Q[j]
            dP = inv_matrix(traj_P[i]) @ traj_P[j]
            rpe = inv_matrix(dQ) @ dP
            err_t = math.hypot(rpe[0, 2], rpe[1, 2])
            err_r = math.atan2(rpe[1, 0], rpe[0, 0])
            errs_trans.append(err_t)
            errs_rot.append(abs(err_r))

        return (
            float(np.mean(errs_trans)),
            float(np.mean(errs_rot)),
            float(np.std(errs_trans)),
            float(np.std(errs_rot)),
        )


def main():
    rclpy.init()
    node = T4Selfcheck()

    print('Waiting for /odometry/filtered and /odom/ground_truth...')
    start = time.time()
    while node.filtered is None or node.ground_truth is None:
        if time.time() - start > 15.0:
            print('FAIL: timeout waiting for topics')
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)

    print('Collecting trajectory 10 sec (vx=0.25 m/s)...')
    traj_P, traj_Q = node.collect_trajectory(vx=0.25, duration=10.0, hz=20.0)
    print('Collected samples: %d' % len(traj_P))

    if len(traj_P) < 20:
        print('FAIL: too few samples (%d)' % len(traj_P))
        return 1

    hz = 20.0
    print('')
    print('RPE (Relative Pose Error):')
    for delta_sec in [0.5, 1.0, 2.0]:
        delta = int(delta_sec * hz)
        res = node.compute_rpe(traj_P, traj_Q, delta)
        if res is None:
            print('  interval %.1fs: not enough samples' % delta_sec)
            continue
        mean_t, mean_r, std_t, std_r = res
        print('  interval %.1fs: trans_mean=%.4f m trans_std=%.4f m rot_mean=%.3f deg rot_std=%.3f deg'
              % (delta_sec, mean_t, std_t, math.degrees(mean_r), math.degrees(std_r)))

    delta = int(1.0 * hz)
    mean_t, mean_r, _, _ = node.compute_rpe(traj_P, traj_Q, delta)
    dist_per_delta = 0.25 * 1.0
    err_pct = mean_t / dist_per_delta * 100.0

    print('')
    print('Verdict for interval 1.0s:')
    print('  translation error = %.2f percent of distance' % err_pct)
    print('  rotation error = %.3f deg' % math.degrees(mean_r))

    if err_pct > 20.0:
        print('FAIL: RPE %.2f percent > 20 percent' % err_pct)
        return 1
    if math.degrees(mean_r) > 10.0:
        print('FAIL: rotation error %.2f deg > 10 deg' % math.degrees(mean_r))
        return 1

    print('OK: T4 selfcheck passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())