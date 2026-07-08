#!/usr/bin/env python3
"""
Goal bridge for Droidal: /goal_pose -> Nav2 NavigateToPose.

This is the single entry point for "go here" commands. It keeps the exact
Foxglove click-to-go workflow (Publish -> Pose on /goal_pose) working with the
full Nav2 stack, and is the same topic the future Android app can publish to.

What it does:
  * Subscribes geometry_msgs/PoseStamped on /goal_pose.
  * Foxglove often stamps the pose in the 3D panel's display frame (e.g.
    'base_link'), not 'map'. We transform the goal (position + heading) into the
    'map' frame via TF so it "just works" regardless of the clicked frame.
  * Sends it as a nav2_msgs/action/NavigateToPose goal. Sending a new goal
    naturally preempts the previous one.
  * Publish std_msgs/Empty on /goal_pose/cancel to cancel the active goal.

Replaces the old goto_goal.py open-loop driver: Nav2 now does planning, obstacle
avoidance and recovery. No angular_sign hack is needed because the base is now
REP-103 compliant.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
import tf2_ros


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GoalBridge(Node):
    def __init__(self):
        super().__init__("goal_bridge")

        self.declare_parameter("map_frame", "map")
        # Foxglove's "2D Goal Pose" tool publishes to /move_base_simple/goal by
        # default, so we listen there. (Nav2's bt_navigator already subscribes to
        # /goal_pose directly, so we deliberately do NOT also listen on /goal_pose
        # -- that would fire two NavigateToPose goals per click.)
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("action_name", "navigate_to_pose")

        self.map_frame = self.get_parameter("map_frame").value
        action_name = self.get_parameter("action_name").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._client = ActionClient(self, NavigateToPose, action_name)
        self._goal_handle = None

        self.create_subscription(
            PoseStamped, self.get_parameter("goal_topic").value,
            self._on_goal, QoSProfile(depth=1))
        self.create_subscription(
            Empty, "/goal_pose/cancel", self._on_cancel, QoSProfile(depth=1))

        self.get_logger().info(
            f"goal_bridge ready: publish PoseStamped on "
            f"'{self.get_parameter('goal_topic').value}' -> Nav2 '{action_name}'")

    # --- goal handling -------------------------------------------------------
    def _on_goal(self, msg: PoseStamped):
        goal_pose = self._to_map(msg)
        if goal_pose is None:
            self.get_logger().warning(
                f"can't transform goal from '{msg.header.frame_id}' to "
                f"'{self.map_frame}' (no TF yet?); ignoring",
                throttle_duration_sec=5.0)
            return

        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning(
                "Nav2 action server not available yet; ignoring goal",
                throttle_duration_sec=5.0)
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        self.get_logger().info(
            f"sending goal: x={goal_pose.pose.position.x:.2f} "
            f"y={goal_pose.pose.position.y:.2f} (map)")
        send_future = self._client.send_goal_async(nav_goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning("Nav2 rejected the goal")
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        # status: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        status = future.result().status
        if status == 4:
            self.get_logger().info("goal reached")
        elif status == 6:
            self.get_logger().warning("goal aborted by Nav2")
        self._goal_handle = None

    def _on_cancel(self, _msg):
        if self._goal_handle is not None:
            self.get_logger().info("cancelling active goal")
            self._goal_handle.cancel_goal_async()
        else:
            self.get_logger().info("cancel requested but no active goal")

    # --- TF ------------------------------------------------------------------
    def _to_map(self, msg: PoseStamped):
        """Return a PoseStamped in the map frame (position + yaw transformed)."""
        frame = msg.header.frame_id or self.map_frame
        x, y = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        if frame != self.map_frame:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None
            tr = t.transform.translation
            tq = t.transform.rotation
            tyaw = _yaw_from_quat(tq.x, tq.y, tq.z, tq.w)
            c, s = math.cos(tyaw), math.sin(tyaw)
            x, y = tr.x + x * c - y * s, tr.y + x * s + y * c
            yaw = yaw + tyaw

        out = PoseStamped()
        out.header.frame_id = self.map_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        return out


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
