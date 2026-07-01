#!/usr/bin/env python3
"""
Lightweight "go to goal" driver for Droidal (Phase 2a; Nav2 is the planned
upgrade for path planning + obstacle avoidance).

Subscribes to a goal pose (geometry_msgs/PoseStamped on /goal_pose -- e.g. from
Foxglove's "Publish -> Pose" tool with the 3D panel's fixed frame set to 'map'),
looks up the robot's pose in the map from TF (map -> base_link), and drives the
diff-drive base toward the goal by publishing geometry_msgs/Twist on /cmd_vel.

Behaviour: rotate toward the goal, then drive forward while correcting heading,
and stop within a tolerance. It does NOT avoid obstacles. Speeds are
deliberately conservative (the robot is heavy); tune via parameters.

cmd_vel units: the ESP32 maps Twist linear.x / angular.z onto ODrive motor
velocity (turns/s), so max_linear / max_angular are in ODrive velocity units,
not m/s. Safety nets: the ESP's cmd_vel watchdog zeroes the motors if this node
stops publishing, and a goal at the robot's own location stops it.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty
import tf2_ros


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GotoGoal(Node):
    def __init__(self):
        super().__init__("goto_goal")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("goal_tolerance", 0.20)      # metres
        self.declare_parameter("heading_threshold", 0.5)    # rad: rotate in place above this
        self.declare_parameter("heading_deadband", 0.12)    # rad: no steering correction below this (anti-oscillation)
        self.declare_parameter("max_linear", 0.4)           # ODrive turns/s (not m/s)
        self.declare_parameter("min_linear", 0.22)          # floor so the heavy base doesn't stall short of the goal
        self.declare_parameter("max_angular", 0.5)
        self.declare_parameter("linear_gain", 0.6)
        self.declare_parameter("angular_gain", 0.9)
        # Measured on the robot: a POSITIVE cmd_vel angular.z makes the odom/map yaw
        # DECREASE (the drive vs odometry rotation convention is opposite). So the
        # steering command must be inverted, otherwise the heading loop is positive
        # feedback and oscillates. Kept as a param in case the firmware is fixed later.
        self.declare_parameter("angular_sign", -1.0)
        self.declare_parameter("max_goal_distance", 30.0)   # ignore clearly bogus clicks

        g = self.get_parameter
        self.map_frame = g("map_frame").value
        self.base_frame = g("base_frame").value
        self.goal_tol = g("goal_tolerance").value
        self.head_thr = g("heading_threshold").value
        self.head_db = g("heading_deadband").value
        self.max_lin = g("max_linear").value
        self.min_lin = g("min_linear").value
        self.max_ang = g("max_angular").value
        self.k_lin = g("linear_gain").value
        self.k_ang = g("angular_gain").value * g("angular_sign").value
        self.max_goal_dist = g("max_goal_distance").value

        self.goal = None  # (x, y) in map frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, g("cmd_topic").value, 10)
        self.create_subscription(
            PoseStamped, g("goal_topic").value, self._on_goal, QoSProfile(depth=1))
        # Abort the current drive: publish std_msgs/Empty on /goto_goal/cancel.
        self.create_subscription(Empty, "/goto_goal/cancel", self._on_cancel, QoSProfile(depth=1))

        period = 1.0 / max(1.0, g("rate_hz").value)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"goto_goal ready: publish a PoseStamped on '{g('goal_topic').value}' "
            f"(frame '{self.map_frame}') to drive there")

    def _on_goal(self, msg):
        # Foxglove stamps published poses with the 3D panel's display frame, which
        # is often 'base_link' (robot-centric view), not 'map'. Accept goals in any
        # frame and transform them into the map via TF so it "just works".
        frame = msg.header.frame_id or self.map_frame
        x, y = msg.pose.position.x, msg.pose.position.y
        if frame != self.map_frame:
            pt = self._to_map(x, y, frame)
            if pt is None:
                self.get_logger().warning(
                    f"can't transform goal from '{frame}' to '{self.map_frame}' "
                    f"(no TF yet?); ignoring", throttle_duration_sec=5.0)
                return
            x, y = pt
        self.goal = (x, y)
        self.get_logger().info(
            f"new goal: x={self.goal[0]:.2f} y={self.goal[1]:.2f} (from '{frame}')")

    def _to_map(self, x, y, src_frame):
        """Transform a 2D point from src_frame into the map frame using latest TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, src_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        return (tr.x + x * c - y * s, tr.y + x * s + y * c)

    def _on_cancel(self, _msg):
        if self.goal is not None:
            self.get_logger().info("goal cancelled")
        self.goal = None
        self._stop()

    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(
                f"no {self.map_frame}->{self.base_frame} TF: {e}",
                throttle_duration_sec=5.0)
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        return tr.x, tr.y, _yaw_from_quat(q.x, q.y, q.z, q.w)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _tick(self):
        if self.goal is None:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        gx, gy = self.goal
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)

        if dist > self.max_goal_dist:
            self.get_logger().warning(
                f"goal {dist:.1f} m away (> {self.max_goal_dist} m); ignoring")
            self.goal = None
            self._stop()
            return

        if dist < self.goal_tol:
            self.get_logger().info(f"goal reached (within {self.goal_tol:.2f} m)")
            self.goal = None
            self._stop()
            return

        yaw_err = _norm(math.atan2(dy, dx) - ryaw)
        cmd = Twist()
        if abs(yaw_err) > self.head_thr:
            # Heading is off: rotate in place before translating.
            cmd.angular.z = _clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)
        else:
            cmd.linear.x = _clamp(self.k_lin * dist, -self.max_lin, self.max_lin)
            # Floor the speed: the heavy base stalls below ~min_lin, which would
            # leave it creeping to a halt short of the goal.
            if 0.0 < cmd.linear.x < self.min_lin:
                cmd.linear.x = self.min_lin
            # Deadband the steering correction so small heading errors don't make
            # the heavy base hunt/oscillate while driving forward.
            if abs(yaw_err) > self.head_db:
                cmd.angular.z = _clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GotoGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
