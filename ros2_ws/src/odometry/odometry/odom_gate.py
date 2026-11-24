#!/usr/bin/env python3
# T4: шлюз визуальной одометрии с удержанием позы при покое робота.
#
# Зачем: RTAB-Map на неподвижном роботе продолжает чуть-чуть уползать
# (шум признаков на кадрах), а критерий T4 требует дрейф не больше
# 0,10 м и 5 градусов за 10 секунд покоя. На стенде уход доходил до
# 0,22 м при реально стоящем роботе (ground truth смещался на 2 мм).
#
# Как работает: узел слушает /visual_odom, /imu/data и /joint_states.
# Покой = норма гироскопа и скорости суставов ниже порогов, удержавшаяся
# REST_CONFIRM секунд. В покое на /visual_odom_gated таймером уходит
# замороженная поза с нулевой скоростью. При возобновлении движения
# оценка продолжается со смещением на накопленный уход RTAB-Map, поэтому
# переход без скачков и относительная точность (RPE) не меняется.
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

GYRO_REST = 0.02      # рад/с: порог нормы гироскопа
JOINT_REST = 0.05     # рад/с: порог скорости суставов
REST_CONFIRM = 0.3    # с: сколько покой должен продержаться до фиксации
HOLD_HZ = 30.0        # Гц: частота публикации замороженной позы


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_of_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def valid_pose(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if any(math.isnan(v) or math.isinf(v) for v in vals):
        return False
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return abs(n - 1.0) < 0.01


class OdomGate(Node):

    def __init__(self):
        super().__init__('odom_gate')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/visual_odom', self.on_vo, qos)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos)
        self.pub = self.create_publisher(Odometry, '/visual_odom_gated', 10)
        self.timer = self.create_timer(1.0 / HOLD_HZ, self.tick)

        self.gyro_norm = None       # норма угловой скорости из /imu/data
        self.joint_speed = None     # максимальная |скорость| суставов
        self.vo_pose = None         # (x, y, yaw) последнего валидного входа
        self.vo_stamp = None
        self.frozen = None          # (x, y, yaw) удерживаемой позы
        self.rest_since = None      # сим-время начала текущего окна покоя
        self.offset = (0.0, 0.0, 0.0)  # x, y, yaw: поправка после удержания
        self.holding = False        # для лога только о сменах состояния

    def on_imu(self, msg):
        g = msg.angular_velocity
        self.gyro_norm = math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z)

    def on_joints(self, msg):
        speeds = [abs(v) for v in msg.velocity if not math.isnan(v)]
        self.joint_speed = max(speeds) if speeds else None

    def at_rest(self):
        return (self.gyro_norm is not None and self.gyro_norm < GYRO_REST
                and self.joint_speed is not None and self.joint_speed < JOINT_REST)

    def on_vo(self, msg):
        if not valid_pose(msg):
            return
        p = msg.pose.pose.position
        self.vo_pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        self.vo_stamp = msg.header.stamp
        if self.frozen is None:
            self.emit(self.vo_pose, self.offset, self.vo_stamp)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.at_rest():
            if self.rest_since is None:
                self.rest_since = t
            if (self.frozen is None and self.vo_pose is not None
                    and t - self.rest_since >= REST_CONFIRM):
                self.frozen = self.vo_pose
            if self.frozen is not None:
                self.emit(self.frozen, (0.0, 0.0, 0.0), None)
                if not self.holding:
                    self.holding = True
                    self.get_logger().info(
                        'rest detected: pose held at (%.3f, %.3f, %.3f rad)'
                        % self.frozen)
        else:
            self.rest_since = None
            if self.frozen is not None:
                # выход из покоя: уход RTAB-Map за время удержания
                # становится поправкой, а не скачком выходной позы
                f, v = self.frozen, self.vo_pose
                if v is not None:
                    self.offset = (f[0] - v[0], f[1] - v[1], wrap(f[2] - v[2]))
                self.frozen = None
                self.holding = False
                self.get_logger().info(
                    'motion resumed: offset (%.3f, %.3f, %.3f rad)'
                    % self.offset)

    def emit(self, pose, off, stamp):
        x = pose[0] + off[0]
        y = pose[1] + off[1]
        yaw = wrap(pose[2] + off[2])
        m = Odometry()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'trunk'
        if stamp is not None:
            m.header.stamp = stamp
        else:
            now = self.get_clock().now().to_msg()
            m.header.stamp = now
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.position.z = 0.0
        _, _, qz, qw = quat_of_yaw(yaw)
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
