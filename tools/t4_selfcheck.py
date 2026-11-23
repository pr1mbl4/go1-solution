#!/usr/bin/env python3
"""T4 selfcheck: RPE (Relative Pose Error).

Без --drive: собирает траекторию без управления (Judge двигает).
С --drive:   публикует /cmd_vel, едет, собирает, останавливается
             (только для локального теста, НЕ для сдачи).

Запуск:
    python3 t4_selfcheck.py            # для сдачи
    python3 t4_selfcheck.py --drive    # локальный тест
"""
import argparse
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
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])


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
    def __init__(self, drive=False, vx=0.25):
        super().__init__('t4_selfcheck')
        self.filtered = None
        self.ground_truth = None
        self.drive = drive
        self.vx = vx

        if self.drive:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_filtered, qos)
        self.create_subscription(Odometry, '/odom/ground_truth', self.on_gt, qos)

    def on_filtered(self, msg):
        self.filtered = msg

    def on_gt(self, msg):
        self.ground_truth = msg

    def collect_trajectory(self, duration=20.0, hz=20.0):
        traj_P, traj_Q = [], []
        dt = 1.0 / hz
        n_steps = int(duration * hz)
        cmd = Twist()
        cmd.linear.x = self.vx

        for i in range(n_steps):
            if self.drive:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=dt)
            if self.filtered is not None and self.ground_truth is not None:
                P = pose_from_msg(self.filtered)
                Q = pose_from_msg(self.ground_truth)
                traj_P.append(pose_to_matrix(*P))
                traj_Q.append(pose_to_matrix(*Q))
            if i % 40 == 0 and self.ground_truth is not None:
                print('  t=%.1fs gt.x=%.3f' % (
                    i * dt, self.ground_truth.pose.pose.position.x))

        if self.drive:
            cmd.linear.x = 0.0
            for _ in range(20):
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

        return traj_P, traj_Q

    def compute_rpe_by_distance(self, traj_P, traj_Q, segment_len=1.0):
        errs_t, errs_r = [], []
        start_idx, acc = 0, 0.0

        for i in range(1, len(traj_Q)):
            dx = traj_Q[i][0, 2] - traj_Q[i-1][0, 2]
            dy = traj_Q[i][1, 2] - traj_Q[i-1][1, 2]
            acc += math.hypot(dx, dy)

            if acc >= segment_len:
                dQ = inv_matrix(traj_Q[start_idx]) @ traj_Q[i]
                dP = inv_matrix(traj_P[start_idx]) @ traj_P[i]
                rpe = inv_matrix(dQ) @ dP
                errs_t.append(math.hypot(rpe[0, 2], rpe[1, 2]))
                errs_r.append(abs(math.atan2(rpe[1, 0], rpe[0, 0])))
                start_idx, acc = i, 0.0

        if not errs_t:
            return None

        return (float(np.mean(errs_t)), float(np.mean(errs_r)),
                float(np.std(errs_t)), float(np.std(errs_r)), len(errs_t))


def main():
    parser = argparse.ArgumentParser(description='T4 selfcheck')
    parser.add_argument('--drive', action='store_true',
                        help='публиковать /cmd_vel (только для локального теста)')
    parser.add_argument('--duration', type=float, default=20.0,
                        help='длительность сбора, сек (по умолчанию 20)')
    parser.add_argument('--vx', type=float, default=0.25,
                        help='скорость при --drive, м/с (по умолчанию 0.25)')
    parser.add_argument('--segment', type=float, default=1.0,
                        help='длина сегмента RPE, м (по умолчанию 1.0)')
    args = parser.parse_args()

    rclpy.init()
    node = T4Selfcheck(drive=args.drive, vx=args.vx)

    if args.drive:
        print('MODE: local test with /cmd_vel (NOT for submission)')
    else:
        print('MODE: submission (no /cmd_vel, Judge drives)')

    print('Waiting for /odometry/filtered and /odom/ground_truth...')
    start = time.time()
    while node.filtered is None or node.ground_truth is None:
        if time.time() - start > 15.0:
            print('FAIL: timeout waiting for topics')
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)

    print('Collecting trajectory %.0f sec...' % args.duration)
    traj_P, traj_Q = node.collect_trajectory(duration=args.duration, hz=20.0)
    print('Collected samples: %d' % len(traj_P))

    if len(traj_P) < 20:
        print('FAIL: too few samples (%d)' % len(traj_P))
        return 1

    res = node.compute_rpe_by_distance(traj_P, traj_Q, segment_len=args.segment)
    if res is None:
        print('FAIL: no %.1fm segments' % args.segment)
        return 1

    mean_t, mean_r, std_t, std_r, n_seg = res
    rpe_pct = mean_t / args.segment * 100.0
    print('RPE by distance (segment=%.1fm):' % args.segment)
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