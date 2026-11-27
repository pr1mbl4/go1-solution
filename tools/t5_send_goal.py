#!/usr/bin/env python3
"""T5 send goal: локальная замена Judge при проверке навигации.

Judge выступает Action-клиентом /navigate_to_pose (ТЗ 7.2) — цель нужно
слать именно так, а не топиком /goal_pose (тот слушает goal_relay для RViz).

Запуск внутри контейнера участника (tools/ в образ не копируется):
    docker cp tools/t5_send_goal.py <участник>:/tmp/
    docker exec <участник> bash -c \
      'source /opt/ros/humble/setup.bash && \
       source /opt/go1_participant/ros2_ws/install/setup.bash && \
       python3 /tmp/t5_send_goal.py --x 4.0 --y 0.0 --yaw 0'

Итог судьи оценивает по ground truth (<=0.5 м, <=30 град), SUCCEEDED от
Nav2 само по себе цель не подтверждает — координаты сверять отдельно.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalSender(Node):

    def __init__(self, args):
        super().__init__('t5_goal_sender')
        self.args = args
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def make_goal(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.args.x
        pose.pose.position.y = self.args.y
        pose.pose.orientation.z = math.sin(self.args.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.args.yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def run(self):
        if not self.client.wait_for_server(timeout_sec=self.args.timeout):
            print('FAIL: action server /navigate_to_pose не появился '
                  'за %.0f с' % self.args.timeout)
            return 2

        goal_future = self.client.send_goal_async(self.make_goal())
        rclpy.spin_until_future_complete(self, goal_future,
                                         timeout_sec=self.args.timeout)
        if not goal_future.done() or not goal_future.result().accepted:
            print('FAIL: Nav2 не принял цель')
            return 2

        print('цель принята, едем...')
        t0 = time.monotonic()
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=self.args.max_drive)
        elapsed = time.monotonic() - t0

        if not result_future.done():
            goal_future.result().cancel_goal_async()
            print('FAIL: не доехали за %.0f с (wall)' % self.args.max_drive)
            return 3

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print('SUCCEEDED за %.1f с (wall)' % elapsed)
            return 0
        print('FAIL: статус действия %d за %.1f с (wall)' % (status, elapsed))
        return 3


def main():
    parser = argparse.ArgumentParser(
        description='Отправить навигационную цель как Judge')
    parser.add_argument('--x', type=float, default=4.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='целевой угол, радианы')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='ожидание action server, с')
    parser.add_argument('--max-drive', type=float, default=300.0,
                        dest='max_drive',
                        help='максимум на проезд, с wall')
    args = parser.parse_args()

    rclpy.init()
    node = GoalSender(args)
    try:
        sys.exit(node.run())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
