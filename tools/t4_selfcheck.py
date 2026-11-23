#!/usr/bin/env python3
"""T4 selfcheck: RPE (Relative Pose Error) для /odometry/filtered.

Сравнивает относительное движение между /odometry/filtered и
/odom/ground_truth на участках длиной 1 метр.
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy),
                      1 - 2 * (qy * qy + qz * qz))


def pose_to_matrix(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1],
    ])


def inv_matrix(T):
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

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_filtered, qos)
        self.create_subscription(
            Odometry, '/odom/ground_truth', self.on_gt, qos)

    def on_filtered(self, msg):
        self.filtered = msg

    def on_gt(self, msg):
        self.ground_truth = msg

    def collect_trajectory(self, duration=60.0, hz=20.0):
        """Собираем траекторию без публикации /cmd_vel (Judge двигает)."""
        traj_P = []
        traj_Q = []
        dt = 1.0 / hz
        n_steps = int(duration * hz)
        for _ in range(n_steps):
            rclpy.spin_once(self, timeout_sec=dt)
            if self.filtered is not None and self.ground_truth is not None:
                P = pose_from_msg(self.filtered)
                Q = pose_from_msg(self.ground_truth)
                traj_P.append(pose_to_matrix(*P))
                traj_Q.append(pose_to_matrix(*Q))
        return traj_P, traj_Q

    def compute_rpe_by_distance(self, traj_P, traj_Q, segment_len=1.0):
        """RPE по участкам фиксированной длины (1 м)."""
        errs_trans = []
        errs_rot = []
        start_idx = 0
        accumulated_dist = 0.0

        for i in range(1, len(traj_Q)):
            dx = traj_Q[i][0, 2] - traj_Q[i - 1][0, 2]
            dy = traj_Q[i][1, 2] - traj_Q[i - 1][1, 2]
            accumulated_dist += math.hypot(dx, dy)

            if accumulated_dist >= segment_len:
                dQ = inv_matrix(traj_Q[start_idx]) @ traj_Q[i]
                dP = inv_matrix(traj_P[start_idx]) @ traj_P[i]
                rpe = inv_matrix(dQ) @ dP
                err_t = math.hypot(rpe[0, 2], rpe[1, 2])
                err_r = math.atan2(rpe[1, 0], rpe[0, 0])
                errs_trans.append(err_t)
                errs_rot.append(abs(err_r))
                start_idx = i
                accumulated_dist = 0.0

        if not errs_trans:
            return None

        return (
            float(np.mean(errs_trans)),
            float(np.mean(errs_rot)),
            float(np.std(errs_trans)),
            float(np.std(errs_rot)),
            len(errs_trans),
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

    print('Collecting trajectory 10 sec...')
    traj_P, traj_Q = node.collect_trajectory(duration=10.0, hz=20.0)
    print('Collected samples: %d' % len(traj_P))

    if len(traj_P) < 20:
        print('FAIL: too few samples (%d)' % len(traj_P))
        return 1

    res = node.compute_rpe_by_distance(traj_P, traj_Q, segment_len=1.0)
    if res is None:
        print('FAIL: no 1m segments')
        return 1
    mean_t, mean_r, std_t, std_r, n_seg = res
    rpe_pct = mean_t / 1.0 * 100.0
    print('RPE by distance (segment=1m):')
    print('  segments: %d' % n_seg)
    print('  trans_mean=%.4f m trans_std=%.4f m rot_mean=%.3f deg rot_std=%.3f deg'
          % (mean_t, std_t, math.degrees(mean_r), math.degrees(std_r)))
    print('  RPE = %.2f percent' % rpe_pct)

    if rpe_pct > 20.0:
        print('FAIL: RPE %.2f percent > 20 percent' % rpe_pct)
        return 1
    if math.degrees(mean_r) > 10.0:
        print('FAIL: rotation %.2f deg > 10 deg' % math.degrees(mean_r))
        return 1
    print('OK: T4 selfcheck passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())