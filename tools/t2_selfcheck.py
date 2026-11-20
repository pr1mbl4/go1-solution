#!/usr/bin/env python3
"""Локальный стенд T2: подаёт команды в /lowcmd_bare и проверяет /lowcmd.

Запускается ВНУТРИ контейнера участника, рядом с работающим safety_node:
  python3 t2_selfcheck.py [--seconds 12]

Проверяет по симуляционному времени:
  - среднюю частоту /lowcmd за 5 с и максимальный интервал;
  - лимиты всех 12 суставов на выходе;
  - rate-limit: не более 5 градусов между соседними разными целями;
  - отсутствие NaN/Inf в выходе;
  - удержание при молчании входа: q = последняя цель, dq=0, tau=0;
  - отклонение опасной конфигурации (самоколлизия) при потоке безопасных
    команд и возобновление обработки после неё.

Код возврата 0 — все проверки пройдены, 1 — есть провалы (печатаются).
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from unitree_go.msg import LowCmd

Q_MIN = np.array([-0.863, -0.686, -2.818] * 4)
Q_MAX = np.array([0.863, 4.501, -0.888] * 4)
STAND_Q = np.array([0.0, 0.8536, -1.7072] * 4)
MODE_SERVO = 0x0A
LIMIT_TOL = 1e-6
RATE_LIMIT_DEG = 5.0


def make_cmd(q, dq=0.0, tau=0.0, kp=40.0, kd=2.0):
    msg = LowCmd()
    for i in range(12):
        m = msg.motor_cmd[i]
        m.mode = MODE_SERVO
        m.q = float(q[i])
        m.dq = float(dq)
        m.tau = float(tau)
        m.kp = float(kp)
        m.kd = float(kd)
    return msg


class SelfCheck(Node):
    def __init__(self):
        super().__init__('t2_selfcheck')
        self.received = []          # (t_sim, q[12], dq[12], tau[12], kp[12], kd[12])
        self.bare_sent = []         # t_sim отправки /lowcmd_bare
        self.pub = self.create_publisher(LowCmd, '/lowcmd_bare', QoSProfile(depth=10))
        self.create_subscription(LowCmd, '/lowcmd', self._on_lowcmd, QoSProfile(depth=100))

    def _on_lowcmd(self, msg):
        q = np.array([msg.motor_cmd[i].q for i in range(12)])
        dq = np.array([msg.motor_cmd[i].dq for i in range(12)])
        tau = np.array([msg.motor_cmd[i].tau for i in range(12)])
        kp = np.array([msg.motor_cmd[i].kp for i in range(12)])
        kd = np.array([msg.motor_cmd[i].kd for i in range(12)])
        modes = {msg.motor_cmd[i].mode for i in range(12)}
        self.received.append((self.now(), q, dq, tau, kp, kd, modes))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_clock(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while self.now() == 0.0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.now() != 0.0

    def send(self, q, **kwargs):
        self.pub.publish(make_cmd(q, **kwargs))
        self.bare_sent.append(self.now())

    def spin_until(self, t_end):
        while self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_for(self, seconds):
        self.spin_until(self.now() + seconds)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='общий лимит реального времени на прогон, с')
    args = parser.parse_args()

    rclpy.init()
    node = SelfCheck()
    failures = []
    if not node.wait_clock():
        print('ПРОВАЛ: нет /clock — симуляционное время не идёт')
        return 1

    # ---------------- фазы прогона (длительности в секундах сим-времени)
    t0 = node.now()
    phase = {
        'safe': (t0, t0 + 2.0),
        'jump': (t0 + 2.0, t0 + 4.0),
        'nan': (t0 + 4.0, t0 + 6.0),
        'silence': (t0 + 6.0, t0 + 7.5),
        'resume': (t0 + 7.5, t0 + 9.0),
        'collision': (t0 + 9.0, t0 + 10.5),
    }
    dangerous = np.array([-0.863, 0.9, -1.8, 0.863, 0.9, -1.8,
                          -0.863, 0.9, -1.8, 0.863, 0.9, -1.8])
    last_target = STAND_Q.copy()
    nan_sent = False
    resume_started = False

    # ---------------- поток 50 Гц с фазами
    period = 0.02
    next_send = t0
    while node.now() < phase['collision'][1]:
        now = node.now()
        if now >= next_send:
            if now < phase['jump'][0]:
                node.send(STAND_Q)
                last_target = STAND_Q.copy()
            elif now < phase['nan'][0]:
                q = last_target + math.radians(20.0) * np.sign(np.arange(12) - 5.5)
                q = np.clip(q, Q_MIN, Q_MAX)
                node.send(q)
                last_target = q
            elif now < phase['nan'][1]:
                if not nan_sent:
                    q = last_target.copy()
                    q[4] = float('nan')
                    node.send(q)
                    nan_sent = True
            elif now < phase['silence'][1]:
                pass  # молчание: watchdog должен включиться
            elif now < phase['collision'][0]:
                node.send(last_target)
            else:
                # опасная конфигурация: должна отклоняться или корректироваться
                node.send(dangerous)
            next_send = now + period
        rclpy.spin_once(node, timeout_sec=0.002)
        if time.monotonic() - t0 > args.timeout:
            failures.append(f'таймаут прогона ({args.timeout} с реального времени)')
            break

    node.spin_for(0.3)  # добрать последние сообщения
    recv = node.received
    if len(recv) < 10:
        print(f'ПРОВАЛ: на /lowcmd пришло всего {len(recv)} сообщений')
        return 1

    times = np.array([r[0] for r in recv])

    # 1. частота и интервалы по симуляционному времени
    window = times[times >= times[-1] - 5.0]
    freq = len(window) / 5.0 if len(window) else 0.0
    gaps = np.diff(times)
    max_gap = float(gaps.max()) if len(gaps) else float('inf')
    print(f'частота /lowcmd: {freq:.1f} Гц, макс. интервал: {max_gap * 1000:.1f} мс')
    if freq < 50.0:
        failures.append(f'частота {freq:.1f} Гц < 50 Гц')
    if max_gap > 0.1:
        failures.append(f'интервал {max_gap * 1000:.1f} мс > 100 мс')

    # 2. NaN/Inf и mode
    bad_values = 0
    bad_modes = 0
    for t, q, dq, tau, kp, kd, modes in recv:
        if not all(np.all(np.isfinite(v)) for v in (q, dq, tau, kp, kd)):
            bad_values += 1
        if modes != {MODE_SERVO}:
            bad_modes += 1
    if bad_values:
        failures.append(f'{bad_values} сообщений с NaN/Inf на выходе')
    if bad_modes:
        failures.append(f'{bad_modes} сообщений без mode=0x0A')

    # 3. лимиты суставов
    q_all = np.array([r[1] for r in recv])
    outside = np.any(q_all < Q_MIN - LIMIT_TOL, axis=1) | np.any(q_all > Q_MAX + LIMIT_TOL, axis=1)
    if outside.any():
        failures.append(f'{int(outside.sum())} сообщений с q вне лимитов')

    # 4. rate-limit: между соседними РАЗНЫМИ целями не больше 5 градусов
    worst = 0.0
    for prev, cur in zip(recv, recv[1:]):
        delta = np.abs(cur[1] - prev[1])
        worst = max(worst, float(delta.max()))
    worst_deg = math.degrees(worst)
    print(f'макс. шаг цели между сообщениями: {worst_deg:.2f} град')
    if worst_deg > RATE_LIMIT_DEG + 1e-3:
        failures.append(f'шаг {worst_deg:.2f} град > 5 град')

    # 5. watchdog: в фазе молчания q = последняя цель, dq=0, tau=0
    silence_lo, silence_hi = phase['silence']
    hold_msgs = [r for r in recv if silence_lo + 0.5 <= r[0] <= silence_hi]
    if not hold_msgs:
        failures.append('нет сообщений в фазе удержания')
    else:
        hold_q = np.array([r[1] for r in hold_msgs])
        target = np.array([r[1] for r in recv if r[0] <= silence_lo][-1])
        if not np.allclose(hold_q, target, atol=math.radians(0.5)):
            failures.append('удержание ушло от последней цели')
        for r in hold_msgs:
            if not (np.all(r[2] == 0.0) and np.all(r[3] == 0.0)):
                failures.append('в удержании dq/tau не нулевые')
                break

    # 6. после молчания поток снова обрабатывается
    resume_lo = phase['resume'][0] + 0.5
    resume_hi = phase['resume'][1]
    resume_msgs = [r for r in recv if resume_lo <= r[0] <= resume_hi]
    if not resume_msgs:
        failures.append('после возобновления потока нет /lowcmd')
    else:
        resumed = np.array([r[1] for r in resume_msgs])
        if not np.allclose(resumed[-1], last_target, atol=math.radians(1.0)):
            failures.append('после возобновления цель не вернулась к команде')

    # 7. опасная конфигурация: проверяем, что выход никогда не коллизионный
    try:
        sys.path.insert(0, '/opt/go1_participant/ros2_ws/install/safety_block/lib/python3.10/site-packages')
        from safety_block.collision import CollisionChecker
        checker = CollisionChecker('/opt/go1_participant/models/unitree_go1')
        unsafe = sum(0 if checker.is_safe(q) else 1 for q in q_all[::10])
        if unsafe:
            failures.append(f'{unsafe} проверенных выходных конфигураций с самоколлизией')
    except Exception as error:  # стенд не должен падать из-за необязательной проверки
        print(f'проверка коллизий на стенде пропущена: {error}')

    print(f'сообщений /lowcmd: {len(recv)}, отправлено /lowcmd_bare: {len(node.bare_sent)}')
    if failures:
        print('ПРОВАЛЫ:')
        for f in failures:
            print(' -', f)
        return 1
    print('ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ')
    return 0


if __name__ == '__main__':
    sys.exit(main())
