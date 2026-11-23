"""Утилита для локальных тестов: публикует /cmd_vel.

НЕ входит в T4. Запускается вручную для проверки одометрии,
когда никто в команде не публикует /cmd_vel.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CmdVelTestPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_test_publisher')
        self.declare_parameter('vx', 0.25)
        self.declare_parameter('wz', 0.0)
        self.vx = self.get_parameter('vx').value
        self.wz = self.get_parameter('wz').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ready = False
        self.create_subscription(
            Bool, '/go1_ready', self.on_ready,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(f'cmd_vel_test_publisher: vx={self.vx}, wz={self.wz}')

    def on_ready(self, msg):
        self.ready = msg.data

    def on_timer(self):
        if not self.ready:
            return
        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.wz
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()