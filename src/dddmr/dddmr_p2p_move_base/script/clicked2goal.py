#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dddmr_sys_core.action import PToPMoveBase
from geometry_msgs.msg import PointStamped, PoseStamped
from action_msgs.msg import GoalStatus


class GoalRelay(Node):

    def __init__(self):
        super().__init__("clicked2p2p")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value

        self._action_client = ActionClient(self, PToPMoveBase, "/p2p_move_base")
        self._request_seq = 0
        self._latest_request_seq = 0
        self._goal_pose_sub = self.create_subscription(
            PoseStamped, "goal_pose_3d", self.goal_cb, 10)
        self._clicked_point_sub = self.create_subscription(
            PointStamped, "clicked_point", self.clicked_point_cb, 10)

    def goal_cb(self, msg: PoseStamped):
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose = msg.pose
        if not goal_pose.header.frame_id:
            goal_pose.header.frame_id = self.global_frame
        if (
            goal_pose.pose.orientation.x == 0.0 and
            goal_pose.pose.orientation.y == 0.0 and
            goal_pose.pose.orientation.z == 0.0 and
            goal_pose.pose.orientation.w == 0.0
        ):
            goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            "Got 3D goal at: %.2f, %.2f, %.2f in %s" % (
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
                goal_pose.header.frame_id,
            )
        )
        self.send_goal(goal_pose)

    def clicked_point_cb(self, msg: PointStamped):
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.header.frame_id = msg.header.frame_id or self.global_frame
        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.pose.position.z = msg.point.z
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            "Got clicked point at: %.2f, %.2f, %.2f in %s" % (
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
                goal_pose.header.frame_id,
            )
        )
        self.send_goal(goal_pose)

    def send_goal(self, pose: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("p2p_move_base action server is not available yet.")
            return

        goal = PToPMoveBase.Goal()
        goal.target_pose = pose
        self.get_logger().info("Sending goal to /p2p_move_base")
        self._request_seq += 1
        request_seq = self._request_seq
        self._latest_request_seq = request_seq
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda future, seq=request_seq: self.goal_response_callback(future, seq))

    def goal_response_callback(self, future, request_seq):
        if request_seq != self._latest_request_seq:
            self.get_logger().info(
                "Ignoring goal response for superseded request #%d" % request_seq)
            return
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, seq=request_seq: self.goal_result_callback(future, seq))

    def goal_result_callback(self, future, request_seq):
        if request_seq != self._latest_request_seq:
            self.get_logger().info(
                "Ignoring goal result for superseded request #%d" % request_seq)
            return
        result = future.result()
        if result is None:
            self.get_logger().warn("Goal result is empty")
            return
        self.get_logger().info("Goal finished with status %d" % result.status)
        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Latest goal was aborted before navigation completed")


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
