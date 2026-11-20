"""ROS 2-узел Safety Block (T2): /lowcmd_bare -> проверки -> /lowcmd >= 50 Гц."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from unitree_go.msg import LowCmd

from safety_block import logic
from safety_block.collision import CollisionChecker


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_block')
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('watchdog_timeout_s', logic.WATCHDOG_TIMEOUT_S)
        self.declare_parameter('rate_limit_deg', 4.99)
        self.declare_parameter('collision_check', True)
        self.declare_parameter('model_dir', '/opt/go1_participant/models/unitree_go1')
        self.declare_parameter('hold_kp', logic.STAND_KP)
        self.declare_parameter('hold_kd', logic.STAND_KD)

        checker = None
        if bool(self.get_parameter('collision_check').value):
            model_dir = str(self.get_parameter('model_dir').value)
            try:
                checker = CollisionChecker(model_dir)
                self.get_logger().info(
                    f'проверка самоколлизий включена: {len(checker.pairs)} пар, {model_dir}')
            except Exception as error:  # модель может отсутствовать — остальные проверки работают
                self.get_logger().error(f'проверка самоколлизий недоступна: {error}')

        self.core = logic.SafetyCore(
            collision_checker=checker,
            rate_limit_rad=math.radians(float(self.get_parameter('rate_limit_deg').value)),
            watchdog_timeout_s=float(self.get_parameter('watchdog_timeout_s').value),
            hold_kp=float(self.get_parameter('hold_kp').value),
            hold_kd=float(self.get_parameter('hold_kd').value),
        )

        # Публикация надёжная: подписчик симулятора и судьи использует RELIABLE.
        self.pub = self.create_publisher(LowCmd, '/lowcmd', QoSProfile(depth=1))
        # Подписка best-effort принимает и RELIABLE-, и BEST_EFFORT-издателей.
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LowCmd, '/lowcmd_bare', self._on_bare, sub_qos)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self._on_timer)
        self._reported = {}
        self.get_logger().info(
            f'safety_block запущен: /lowcmd_bare -> /lowcmd, {1.0 / period:.0f} Гц, '
            f'watchdog {self.core.watchdog_timeout_s:.2f} с')

    # ---------------------------------------------------------------- вход
    def _on_bare(self, msg):
        q, dq, tau, kp, kd = (
            np.array([msg.motor_cmd[i].q for i in range(logic.N_JOINTS)]),
            np.array([msg.motor_cmd[i].dq for i in range(logic.N_JOINTS)]),
            np.array([msg.motor_cmd[i].tau for i in range(logic.N_JOINTS)]),
            np.array([msg.motor_cmd[i].kp for i in range(logic.N_JOINTS)]),
            np.array([msg.motor_cmd[i].kd for i in range(logic.N_JOINTS)]),
        )
        status = self.core.on_message(self._now(), q, dq, tau, kp, kd)
        if status != logic.ACCEPTED:
            # Не засоряем лог: одно сообщение на каждый тип отказа за прогон.
            if not self._reported.get(status):
                self._reported[status] = True
                self.get_logger().warn(f'команда отклонена: {status}')

    # --------------------------------------------------------------- выход
    def _on_timer(self):
        command = self.core.on_tick(self._now())
        self.pub.publish(self._build_message(command))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _build_message(command):
        msg = LowCmd()
        msg.head = [0xFE, 0xEE]
        msg.level_flag = 0xFF
        for i in range(logic.N_SLOTS):
            motor = msg.motor_cmd[i]
            if i < logic.N_JOINTS:
                motor.mode = logic.MODE_SERVO  # 0x0A обязателен для позиционного управления
                motor.q = float(command.q[i])
                motor.dq = float(command.dq[i])
                motor.tau = float(command.tau[i])
                motor.kp = float(command.kp[i])
                motor.kd = float(command.kd[i])
            else:
                motor.mode = 0
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
