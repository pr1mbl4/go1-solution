"""Hold a PoseStamped goal until the Nav2 navigator can accept it."""

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')
        self.pending_pose = None
        self.state_request = None
        self.navigator_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self.receive_goal, 10)
        self.create_timer(0.25, self.check_navigator)

    def receive_goal(self, pose):
        self.pending_pose = pose

    def check_navigator(self):
        if (self.pending_pose is not None and self.state_request is None
                and self.navigator_state.service_is_ready() and self.navigator.server_is_ready()):
            self.state_request = self.navigator_state.call_async(GetState.Request())
            self.state_request.add_done_callback(self.forward_if_active)

    def forward_if_active(self, future):
        self.state_request = None
        response = future.result()
        if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.pending_pose
        self.pending_pose = None
        self.navigator.send_goal_async(goal).add_done_callback(self.report_acceptance)

    def report_acceptance(self, future):
        if future.result().accepted:
            self.get_logger().info('Nav2 accepted the goal received on /goal_pose')
        else:
            self.get_logger().warning('Nav2 rejected the goal received on /goal_pose')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
