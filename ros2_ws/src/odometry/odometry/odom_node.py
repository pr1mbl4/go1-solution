"""T4: узел одометрии Go1.

Публикует:
  /odom           nav_msgs/Odometry        — поза и скорости
  TF odom -> trunk                          — для CHAMP

Подписан на:
  /imu/data       sensor_msgs/Imu           — ориентация (yaw)
  /cmd_vel        geometry_msgs/Twist       — скорости (от другого узла команды)
  /go1_ready      std_msgs/Bool             — готовность (опционально)

"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from odometry.odom_logic import yaw_from_quaternion, integrate_pose, wrap_angle


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_odom', True)

        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_odom = self.get_parameter('publish_odom').value

        # Поза
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp = None

        # Скорости из /cmd_vel
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.imu_seen = False
        self.cmd_seen = False

        qos_best_effort = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE)

        # Подписки
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_best_effort)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, qos_reliable)
        self.create_subscription(
            Bool, '/go1_ready', self.on_ready,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Публикации
        self.pub_odom = self.create_publisher(Odometry, '/odom_dead_reckoning', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Таймер интегрирования — 50 Гц
        self.create_timer(0.02, self.on_timer)

        self.get_logger().info('odom_node started (odometry only, no cmd_vel publishing)')

    def on_imu(self, msg: Imu):
        self.imu_seen = True
        q = msg.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_cmd(self, msg: Twist):
        self.cmd_seen = True
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def on_ready(self, msg: Bool):
        # Не используется для управления, оставлено для диагностики
        pass

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if dt <= 0.0 or dt > 1.0:
            return

        self.x, self.y, self.yaw = integrate_pose(
            self.x, self.y, self.yaw, self.vx, self.vy, self.wz, dt)
        self.yaw = wrap_angle(self.yaw)

        if self.publish_odom:
            self.publish_odom_msg(now)

    def publish_odom_msg(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'trunk'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        for i in range(36):
            msg.pose.covariance[i] = 0.0
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05

        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz
        for i in range(36):
            msg.twist.covariance[i] = 0.0
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[7] = 0.05
        msg.twist.covariance[35] = 0.05

        self.pub_odom.publish(msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'trunk'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(self.yaw)
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()