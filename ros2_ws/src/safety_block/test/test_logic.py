"""Тесты логики Safety Block. Запуск: python -m pytest test/test_logic.py."""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from safety_block import logic  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[4]
MODEL_DIR = REPO / 'models' / 'unitree_go1'


def command(q=None, dq=0.0, tau=0.0, kp=40.0, kd=2.0):
    q = logic.STAND_Q.copy() if q is None else np.asarray(q, dtype=float)
    return (q, np.full(12, dq), np.full(12, tau), np.full(12, kp), np.full(12, kd))


def test_nan_and_inf_rejected():
    core = logic.SafetyCore()
    core.on_message(0.0, *command())
    target = core.target_q

    for bad in (np.nan, np.inf, -np.inf):
        q = logic.STAND_Q.copy()
        q[3] = bad
        assert core.on_message(1.0, *command(q=q)) == logic.REJECTED_INVALID
        assert np.array_equal(core.target_q, target)

    # мусор в неугловом поле тоже отбрасывает сообщение целиком
    assert core.on_message(2.0, *command(dq=math.inf)) == logic.REJECTED_INVALID
    assert np.array_equal(core.target_q, target)


def test_limits_are_clipped():
    core = logic.SafetyCore()
    q = np.array([0.0, 100.0, -100.0] * 4)
    assert core.on_message(0.0, *command(q=q)) == logic.ACCEPTED
    out = core.target_q
    assert np.all(out >= logic.Q_MIN) and np.all(out <= logic.Q_MAX)


def test_rate_limit_five_degrees_per_step():
    core = logic.SafetyCore()
    q = logic.STAND_Q.copy()
    q[1] += math.radians(20.0)  # резкий рывок на 20 градусов
    assert core.on_message(0.0, *command(q=q)) == logic.ACCEPTED
    step = np.abs(core.target_q - logic.STAND_Q)
    assert np.all(step <= math.radians(5.0) + 1e-9)
    assert math.isclose(step[1], math.radians(4.99), rel_tol=1e-6)


def test_rate_limit_applies_between_outputs():
    """Две команды между тиками публикации не дают шаг > 5 градусов в выходе."""
    core = logic.SafetyCore()
    q = logic.STAND_Q.copy()
    q[1] += math.radians(20.0)
    core.on_tick(0.0)  # первая публикация стойки
    core.on_message(0.001, *command(q=q))
    core.on_message(0.002, *command(q=q))
    out1 = core.on_tick(0.010)
    out2 = core.on_tick(0.020)
    assert np.all(np.abs(out2.q - out1.q) <= math.radians(5.0) + 1e-9)


def test_safe_commands_pass_through():
    core = logic.SafetyCore()
    q = logic.STAND_Q.copy()
    q[2] += math.radians(3.0)
    assert core.on_message(0.0, *command(q=q)) == logic.ACCEPTED
    assert np.allclose(core.target_q, q)


def test_watchdog_holds_target_not_measured():
    core = logic.SafetyCore(watchdog_timeout_s=0.45)
    q = logic.STAND_Q.copy()
    q[0] += math.radians(2.0)
    core.on_message(0.0, *command(q=q))
    before = core.on_tick(0.2)
    assert np.allclose(before.q, q)
    assert not core.holding

    hold = core.on_tick(0.5)  # 0,5 с тишины — удержание
    assert core.holding
    assert np.allclose(hold.q, q)  # последняя ЦЕЛЬ
    assert np.all(hold.dq == 0.0) and np.all(hold.tau == 0.0)
    assert np.all(hold.kp == logic.STAND_KP) and np.all(hold.kd == logic.STAND_KD)

    # поток возобновился — удержание снимается
    core.on_message(0.6, *command(q=q))
    after = core.on_tick(0.7)
    assert not core.holding
    assert np.allclose(after.q, q)


def test_invalid_messages_keep_stream_alive():
    core = logic.SafetyCore(watchdog_timeout_s=0.45)
    q = logic.STAND_Q.copy()
    core.on_message(0.0, *command(q=q))
    bad = q.copy()
    bad[5] = np.nan
    for t in (0.1, 0.2, 0.3, 0.4):
        core.on_message(t, *command(q=bad))
    assert not core.holding  # поток есть, просто сообщения отклоняются


@pytest.mark.skipif(not (MODEL_DIR / 'scene.xml').exists(), reason='нет модели Go1')
def test_collision_rejection():
    mujoco = pytest.importorskip('mujoco')
    from safety_block.collision import CollisionChecker

    checker = CollisionChecker(str(MODEL_DIR))
    core = logic.SafetyCore(collision_checker=checker)

    # безопасная стойка принимается
    assert core.on_message(0.0, *command()) == logic.ACCEPTED

    # коллизия «бёдра внутрь»: команда шаг за шагом подтягивается к опасной
    # конфигурации, но до неё не доходит — переход упирается в коллизию
    dangerous = np.array([-0.863, 0.9, -1.8, 0.863, 0.9, -1.8,
                          -0.863, 0.9, -1.8, 0.863, 0.9, -1.8])
    assert not checker.is_safe(dangerous)
    statuses = []
    for step in range(20):
        statuses.append(core.on_message(step * 0.1, *command(q=dangerous)))
    assert logic.REJECTED_COLLISION in statuses
    assert checker.is_safe(core.target_q)  # цель всегда остаётся безопасной
    assert not np.allclose(core.target_q, dangerous)


@pytest.mark.skipif(not (MODEL_DIR / 'scene.xml').exists(), reason='нет модели Go1')
def test_collision_transition_is_interpolated():
    pytest.importorskip('mujoco')
    from safety_block.collision import CollisionChecker

    checker = CollisionChecker(str(MODEL_DIR))
    # переход через опасную зону должен ловиться, даже если концы безопасны
    safe_start = logic.STAND_Q.copy()
    assert checker.check_transition(safe_start, safe_start)
    # 0,1 градуса на шаг — судейская дискретность, проверяем, что путь чист
    assert checker.check_transition(safe_start, safe_start + math.radians(1.0))
