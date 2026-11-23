import math
import pytest
from odometry.odom_logic import yaw_from_quaternion, integrate_pose, wrap_angle


def test_yaw_identity():
    # Кватернион (0,0,0,1) -> yaw 0
    assert abs(yaw_from_quaternion(0, 0, 0, 1)) < 1e-9


def test_yaw_90deg():
    # Кватернион для поворота на 90° вокруг Z:
    # z = sin(45°) = sqrt(2)/2, w = cos(45°) = sqrt(2)/2
    s = math.sqrt(2) / 2
    yaw = yaw_from_quaternion(0, 0, s, s)
    assert abs(yaw - math.pi / 2) < 1e-9


def test_integrate_forward():
    x, y, yaw = integrate_pose(0, 0, 0, 1.0, 0.0, 0.0, 1.0)
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 0.0) < 1e-9
    assert abs(yaw - 0.0) < 1e-9


def test_integrate_after_90deg():
    # yaw = 90°, vx = 1 м/с, dt = 1 с -> движение по +Y
    x, y, yaw = integrate_pose(0, 0, math.pi / 2, 1.0, 0.0, 0.0, 1.0)
    assert abs(x - 0.0) < 1e-9
    assert abs(y - 1.0) < 1e-9


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9
    assert abs(wrap_angle(-3 * math.pi) - (-math.pi)) < 1e-9 or \
           abs(wrap_angle(-3 * math.pi) - math.pi) < 1e-9