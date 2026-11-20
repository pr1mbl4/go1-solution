"""Логика Safety Block без зависимости от ROS.

Порядок проверки входящей команды:
  1. конечность всех управляющих полей (q, dq, tau, kp, kd) 12 суставов;
  2. ограничение q лимитами суставов;
  3. rate-limit: не более 5 градусов на сустав между принятыми командами;
  4. самоколлизии на всём переходе к новой цели (шаг 0,05 градуса);
  5. watchdog: при молчании входа дольше порога — удержание последней цели.

Модуль не импортирует rclpy и тестируется обычным pytest.
"""
import math
from collections import Counter
from dataclasses import dataclass

import numpy as np

N_JOINTS = 12
N_SLOTS = 20
MODE_SERVO = 0x0A  # без этого режима симулятор игнорирует q/kp

# Лимиты ТЗ T2, рад: FR,FL,RR,RL x hip,thigh,calf.
Q_MIN = np.array([-0.863, -0.686, -2.818] * 4, dtype=float)
Q_MAX = np.array([0.863, 4.501, -0.888] * 4, dtype=float)

# Поза и коэффициенты мягкого старта симулятора (go1_unitree_bridge.core).
STAND_Q = np.array([0.0, 0.8536, -1.7072] * 4, dtype=float)
STAND_KP = 40.0
STAND_KD = 2.0

# 5 градусов с небольшим запасом, чтобы не упереться в границу при округлении.
RATE_LIMIT_RAD = math.radians(4.99)
# Удержание должно наступить не позднее 0,5 с симуляционного времени.
WATCHDOG_TIMEOUT_S = 0.45

ACCEPTED = "accepted"
REJECTED_INVALID = "rejected_invalid"
REJECTED_COLLISION = "rejected_collision"


@dataclass
class Command:
    """Целевая команда на 12 суставов в порядке ТЗ."""

    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    kp: np.ndarray
    kd: np.ndarray

    def copy(self):
        return Command(self.q.copy(), self.dq.copy(), self.tau.copy(),
                       self.kp.copy(), self.kd.copy())


def stand_command(kp=STAND_KP, kd=STAND_KD):
    return Command(q=STAND_Q.copy(), dq=np.zeros(N_JOINTS), tau=np.zeros(N_JOINTS),
                   kp=np.full(N_JOINTS, kp), kd=np.full(N_JOINTS, kd))


def _finite(values):
    return np.all(np.isfinite(np.asarray(values, dtype=float)))


class SafetyCore:
    """Конечный автомат безопасности: вход — команды и время, выход — команда."""

    def __init__(self, collision_checker=None, rate_limit_rad=RATE_LIMIT_RAD,
                 watchdog_timeout_s=WATCHDOG_TIMEOUT_S, hold_kp=STAND_KP, hold_kd=STAND_KD):
        self.collision_checker = collision_checker
        self.rate_limit_rad = float(rate_limit_rad)
        self.watchdog_timeout_s = float(watchdog_timeout_s)
        self.hold_kp = float(hold_kp)
        self.hold_kd = float(hold_kd)

        self._target = stand_command()
        self._last_bare_time = None
        self._holding = False
        self.counters = Counter()

    # ------------------------------------------------------------------ вход
    def on_message(self, now, q, dq, tau, kp, kd):
        """Обработать входящую команду. Возвращает статус ACCEPTED/REJECTED_*.

        Любое сообщение (даже отклонённое) подтверждает, что поток жив:
        watchdog реагирует только на полное отсутствие сообщений.
        """
        self._last_bare_time = now
        fields = [np.asarray(v, dtype=float).reshape(-1) for v in (q, dq, tau, kp, kd)]
        if any(v.size < N_JOINTS for v in fields):
            self.counters[REJECTED_INVALID] += 1
            return REJECTED_INVALID
        q, dq, tau, kp, kd = (v[:N_JOINTS] for v in fields)

        if not all(_finite(v) for v in (q, dq, tau, kp, kd)):
            self.counters[REJECTED_INVALID] += 1
            return REJECTED_INVALID

        # Лимиты суставов: опасное значение корректируется до допустимого.
        q_clipped = np.clip(q, Q_MIN, Q_MAX)
        # Rate-limit: за один принятый шаг не дальше 5 градусов на сустав.
        q_step = np.clip(q_clipped, self._target.q - self.rate_limit_rad,
                         self._target.q + self.rate_limit_rad)

        if self.collision_checker is not None:
            # Повтор той же цели не требует перепроверки перехода.
            if not np.array_equal(q_step, self._target.q):
                if not self.collision_checker.check_transition(self._target.q, q_step):
                    self.counters[REJECTED_COLLISION] += 1
                    return REJECTED_COLLISION

        self._target = Command(q=q_step, dq=dq, tau=tau, kp=kp, kd=kd)
        self.counters[ACCEPTED] += 1
        return ACCEPTED

    # ----------------------------------------------------------------- выход
    def on_tick(self, now):
        """Команда для публикации в /lowcmd на текущем шаге таймера."""
        if (self._last_bare_time is not None
                and now - self._last_bare_time >= self.watchdog_timeout_s):
            if not self._holding:
                self._holding = True
                self.counters["watchdog_hold"] += 1
            # Удержание последней принятой ЦЕЛИ, а не измеренного положения.
            return Command(q=self._target.q.copy(), dq=np.zeros(N_JOINTS),
                           tau=np.zeros(N_JOINTS),
                           kp=np.full(N_JOINTS, self.hold_kp),
                           kd=np.full(N_JOINTS, self.hold_kd))
        self._holding = False
        return self._target.copy()

    @property
    def holding(self):
        return self._holding

    @property
    def target_q(self):
        return self._target.q.copy()
