"""Проверка самоколлизий целевых конфигураций Go1 по модели MuJoCo.

Используется эталонная модель unitree_go1 (MuJoCo Menagerie, rev 0059d433)
и перечень проверяемых пар t2_collision_pairs.csv из публичной симуляции.
Запрещённой считается самоколлизия с проникновением строго более 1 мм.
"""
import csv
import math

import numpy as np

PENETRATION_LIMIT_M = 0.001
# Шаг интерполяции перехода: тоньше судейских 0,1 градуса.
DEFAULT_INTERP_STEP_RAD = math.radians(0.05)


def load_checked_pairs(csv_path):
    """Пары геометрий со статусом 'checked' из t2_collision_pairs.csv."""
    pairs = set()
    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["diagnostic_policy"].strip() == "checked":
                g1, g2 = int(row["geom1_id"]), int(row["geom2_id"])
                pairs.add((min(g1, g2), max(g1, g2)))
    if not pairs:
        raise RuntimeError(f"в {csv_path} не найдено ни одной проверяемой пары")
    return pairs


class CollisionChecker:
    """Проверка конфигураций и переходов на самоколлизии.

    Конструктор поднимает MjModel и пары; метод check_transition проверяет
    весь путь между конфигурациями с шагом не грубее 0,1 градуса.
    """

    def __init__(self, model_dir, penetration_limit_m=PENETRATION_LIMIT_M):
        import mujoco  # импорт здесь, чтобы логика работала без mujoco

        self._mujoco = mujoco
        self.penetration_limit_m = float(penetration_limit_m)
        scene = f"{model_dir}/scene.xml"
        pairs_csv = f"{model_dir}/t2_collision_pairs.csv"
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.pairs = load_checked_pairs(pairs_csv)
        if self.model.nq < 19:
            raise RuntimeError(f"неожиданная модель: nq={self.model.nq}, ожидалось 19")

    def check(self, q12):
        """Максимальное проникновение по проверяемым парам для конфигурации q12."""
        q = np.asarray(q12, dtype=float).reshape(-1)
        if q.size != 12:
            raise ValueError(f"ожидалось 12 углов, получено {q.size}")
        data = self.data
        data.qpos[:] = 0.0
        data.qpos[2] = 0.35          # корпус над полом; контакты с полом не входят в пары
        data.qpos[3] = 1.0           # кватернион (w, x, y, z) без наклона
        data.qpos[7:19] = q
        self._mujoco.mj_forward(self.model, data)
        max_pen = 0.0
        for i in range(data.ncon):
            contact = data.contact[i]
            key = (min(contact.geom1, contact.geom2), max(contact.geom1, contact.geom2))
            if key in self.pairs:
                pen = -contact.dist     # dist < 0 — проникновение
                if pen > max_pen:
                    max_pen = pen
        return max_pen

    def is_safe(self, q12):
        return self.check(q12) <= self.penetration_limit_m

    def check_transition(self, q_from, q_to, step_rad=DEFAULT_INTERP_STEP_RAD):
        """True, если весь переход между конфигурациями без запрещённых коллизий."""
        q_from = np.asarray(q_from, dtype=float)
        q_to = np.asarray(q_to, dtype=float)
        delta = float(np.max(np.abs(q_to - q_from)))
        steps = max(1, int(math.ceil(delta / step_rad)))
        for k in range(steps + 1):
            q = q_from + (q_to - q_from) * (k / steps)
            if not self.is_safe(q):
                return False
        return True
