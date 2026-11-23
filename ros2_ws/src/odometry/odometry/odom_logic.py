import math


def yaw_from_quaternion(qx, qy, qz, qw):
    """Yaw (рыскание) из кватерниона, радианы."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def integrate_pose(x, y, yaw, vx, vy, wz, dt):
    """Один шаг интегрирования дифференциального привода.

    x, y, yaw    — текущая поза (м, м, рад)
    vx, vy, wz   — скорости из /cmd_vel (м/с, м/с, рад/с)
    dt           — шаг времени (с)

    Возвращает новую позу (x, y, yaw).
    """
    dx = (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
    dy = (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
    return x + dx, y + dy, yaw + wz * dt


def wrap_angle(angle):
    """Привести угол к диапазону (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle