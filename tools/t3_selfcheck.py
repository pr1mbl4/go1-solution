#!/usr/bin/env python3
"""T3 selfcheck: один эпизод по протоколу судьи (ТЗ T3 п. 5.3).

Воспроизводит сценарий испытания локально: Judge заменяется этим скриптом
(в судейской среде /cmd_vel публикует Judge, участнику публиковать запрещено —
здесь это локальная имитация для стенда, НЕ часть решения).

Протокол: прогрев 1,5 с сим-времени (cmd_vel уже льётся) -> зачётное движение
до 90 с сим-времени. Успех: вперёд >= 3 м от начального направления по
/odom/ground_truth без падения. Падение: высота корпуса < 0,16 м либо
|roll|, |pitch| > 50° непрерывно более 0,5 с сим-времени (сброс при возврате).
Все окна — по сим-времени /clock (RTF стенда не влияет).

Запуск внутри сим-контейнера (rclpy есть только там):
    docker cp tools/t3_selfcheck.py go1-local-<id>-sim:/tmp/
    docker exec go1-local-<id>-sim python3 /tmp/t3_selfcheck.py --speed 0.3

Выход: 0 — эпизод успешен, 1 — провал (падение, дистанция, таймаут, стык).
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from unitree_go.msg import LowState, LowCmd

FALL_HEIGHT_M = 0.16
FALL_ANGLE_RAD = math.radians(50.0)
VIOLATION_S = 0.5
LOWCMD_MIN_HZ = 50.0
LOWCMD_MAX_GAP_S = 0.1


def roll_pitch_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class EpisodeRunner(Node):
    def __init__(self, speed, distance, warmup_s, limit_s, lowcmd_check):
        super().__init__('t3_selfcheck')
        self.speed = speed
        self.distance = distance
        self.warmup_s = warmup_s
        self.limit_s = limit_s
        self.lowcmd_check = lowcmd_check

        self.sim_t = None          # сим-время из /clock
        self.gt = None             # последнее /odom/ground_truth
        self.lowstate_seen = False

        self.t_first_cmd = None    # сим-время первой публикации /cmd_vel
        self.t_start = None        # сим-время старта зачёта
        self.start_pose = None     # (x, y, yaw) на старте зачёта
        self.result = None         # ('SUCCESS', ...) | ('FAIL', причина, ...)

        # нарушение: [since, ...] по трём условиям (высота, roll, pitch)
        self.violation_since = [None, None, None]

        # частоты стыка: [(сим-время прибытия), ...]
        self.lowcmd_stamps = []
        self.bare_stamps = []

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_be)
        self.create_subscription(Odometry, '/odom/ground_truth', self.on_gt, qos_be)
        self.create_subscription(LowState, '/lowstate', self.on_lowstate, qos_be)
        self.create_subscription(LowCmd, '/lowcmd', self.on_lowcmd, qos_be)
        self.create_subscription(LowCmd, '/lowcmd_bare', self.on_bare, qos_be)
        # публикация RELIABLE по умолчанию: принимается и RELIABLE-, и BE-читателями
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_gt(self, msg):
        self.gt = msg
        if self.t_start is not None:
            self.check_fall()

    def on_lowstate(self, _msg):
        self.lowstate_seen = True

    def on_lowcmd(self, _msg):
        if self.lowcmd_check:
            self.lowcmd_stamps.append(self.sim_t)

    def on_bare(self, _msg):
        if self.lowcmd_check:
            self.bare_stamps.append(self.sim_t)

    # ---------------------------------------------------------------- падение
    def check_fall(self):
        p = self.gt.pose.pose.position
        q = self.gt.pose.pose.orientation
        roll, pitch = roll_pitch_from_quat(q.x, q.y, q.z, q.w)
        conds = (p.z < FALL_HEIGHT_M,
                 abs(roll) > FALL_ANGLE_RAD,
                 abs(pitch) > FALL_ANGLE_RAD)
        for i, bad in enumerate(conds):
            if bad:
                if self.violation_since[i] is None:
                    self.violation_since[i] = self.sim_t
                elif self.sim_t - self.violation_since[i] > VIOLATION_S:
                    names = ('height %.3f m' % p.z,
                             'roll %.1f deg' % math.degrees(roll),
                             'pitch %.1f deg' % math.degrees(pitch))
                    self.result = ('FAIL', 'fall: %s held > %.1f s' % (names[i], VIOLATION_S))
                    return
            else:
                self.violation_since[i] = None

    # ---------------------------------------------------------------- движение
    def progress(self):
        """(forward, lateral, время) от начального направления."""
        p = self.gt.pose.pose.position
        x0, y0, yaw0 = self.start_pose
        dx, dy = p.x - x0, p.y - y0
        forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        return forward, lateral, self.sim_t - self.t_start

    def tick(self):
        """Один шаг логики эпизода. Возвращает True, когда эпизод завершён."""
        if self.gt is None or self.sim_t is None:
            return False

        cmd = Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)
        if self.t_first_cmd is None:
            self.t_first_cmd = self.sim_t

        if self.t_start is None:
            # прогрев: cmd_vel уже идёт, зачёт стартует через warmup сим-секунд
            if self.sim_t - self.t_first_cmd >= self.warmup_s and self.lowstate_seen:
                p = self.gt.pose.pose.position
                q = self.gt.pose.pose.orientation
                yaw0 = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.start_pose = (p.x, p.y, yaw0)
                self.t_start = self.sim_t
                print('  warmup done, t=%.2f: start pose x=%.3f y=%.3f yaw0=%.1f deg'
                      % (self.sim_t, p.x, p.y, math.degrees(yaw0)))
            return self.result is not None

        if self.result is not None:
            return True

        forward, lateral, elapsed = self.progress()
        if forward >= self.distance:
            self.result = ('SUCCESS', forward, lateral, elapsed)
            return True
        if elapsed > self.limit_s:
            self.result = ('FAIL', 'distance: %.3f m in %.1f s (limit %d s)'
                           % (forward, elapsed, self.limit_s))
            return True
        return False

    # ---------------------------------------------------------------- стык
    def rate_report(self, stamps, label):
        if not self.lowcmd_check:
            return ''
        if len(stamps) < 2:
            return ' %s: <2 msgs' % label
        gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
        span = stamps[-1] - stamps[0]
        hz = (len(stamps) - 1) / span if span > 0 else 0.0
        max_gap = max(gaps) if gaps else 0.0
        return ' %s: %d msgs, %.1f Hz avg, max gap %.1f ms' % (
            label, len(stamps), hz, max_gap * 1e3)

    def joint_report(self):
        ok = ''
        for stamps, label in ((self.lowcmd_stamps, '/lowcmd'),
                              (self.bare_stamps, '/lowcmd_bare')):
            rep = self.rate_report(stamps, label)
            if rep:
                ok += rep + '\n'
        return ok


def wait_ready(node, wall_timeout=120.0):
    """Ждём /clock, /lowstate и /odom/ground_truth (настенное время)."""
    import time
    t0 = time.monotonic()
    while node.gt is None or node.sim_t is None or not node.lowstate_seen:
        if time.monotonic() - t0 > wall_timeout:
            print('FAIL: timeout waiting for /clock, /lowstate, /odom/ground_truth')
            return False
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


def main():
    parser = argparse.ArgumentParser(description='T3 selfcheck: один эпизод по протоколу судьи')
    parser.add_argument('--speed', type=float, default=0.3,
                        help='linear.x /cmd_vel, м/с (preferred_speed_mps, по умолчанию 0.3)')
    parser.add_argument('--distance', type=float, default=3.0,
                        help='требуемое перемещение вперёд, м (по умолчанию 3.0)')
    parser.add_argument('--warmup', type=float, default=1.5,
                        help='прогрев, сим-с (по умолчанию 1.5)')
    parser.add_argument('--limit', type=float, default=90.0,
                        help='лимит зачётного движения, сим-с (по умолчанию 90)')
    parser.add_argument('--no-lowcmd-check', action='store_true',
                        help='не мерять частоту /lowcmd и /lowcmd_bare')
    args = parser.parse_args()

    rclpy.init()
    node = EpisodeRunner(args.speed, args.distance, args.warmup, args.limit,
                         not args.no_lowcmd_check)
    print('MODE: local episode (this script plays Judge — NOT part of the solution)')
    print('speed=%.2f m/s, distance=%.1f m, warmup=%.1f s, limit=%d s (sim time)'
          % (args.speed, args.distance, args.warmup, args.limit))

    if not wait_ready(node):
        return 1
    print('  topics ready: t=%.2f' % node.sim_t)

    import time
    last_report = -5.0
    wall0 = time.monotonic()
    wall_limit = (args.warmup + args.limit) / 0.15 + 300.0  # запас на RTF ~0,15
    finished = False
    while not finished:
        rclpy.spin_once(node, timeout_sec=0.05)
        finished = node.tick()
        if (node.t_start is not None and node.result is None
                and node.sim_t - node.t_start - last_report >= 5.0):
            last_report = node.sim_t - node.t_start
            forward, lateral, elapsed = node.progress()
            print('  t=%5.1f s: forward=%.3f m, lateral=%.3f m'
                  % (elapsed, forward, lateral))
        if node.sim_t - (node.t_first_cmd or node.sim_t) > (args.warmup + args.limit + 15.0):
            # страховка от зависшего /clock или потери сообщений
            if node.result is None:
                node.result = ('FAIL', 'no verdict: /clock stalled or stream lost')
        if time.monotonic() - wall0 > wall_limit:
            # страховка от мёртвого симулятора: вердикт по настенному времени
            if node.result is None:
                node.result = ('FAIL', 'no verdict: simulator wall-clock timeout')

    kind = node.result[0]
    if kind == 'SUCCESS':
        forward, lateral, elapsed = node.result[1:]
        drift = math.degrees(math.atan2(lateral, forward)) if forward > 1e-6 else 0.0
        print('EPISODE RESULT: SUCCESS')
        print('  forward=%.3f m, lateral=%.3f m, drift=%.1f deg, time=%.1f s (sim)'
              % (forward, lateral, drift, elapsed))
    else:
        print('EPISODE RESULT: FAIL — %s' % node.result[1])
    joint = node.joint_report()
    if joint:
        print(joint, end='')
    if not args.no_lowcmd_check:
        stamps = node.lowcmd_stamps
        span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
        if span > 0:
            hz = (len(stamps) - 1) / span
            if hz < LOWCMD_MIN_HZ:
                # предупреждение оператору: контракт T2 нарушен, но вердикт
                # эпизода T3 не меняет (судья меряет критерии независимо)
                print('JOINT WARNING: /lowcmd %.1f Hz < %d Hz (T2 contract)' % (hz, LOWCMD_MIN_HZ))
    rclpy.shutdown()
    return 0 if kind == 'SUCCESS' else 1


if __name__ == '__main__':
    sys.exit(main())
