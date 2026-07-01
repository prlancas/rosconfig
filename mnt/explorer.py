#!/usr/bin/env python3
"""
Frontier-based autonomous exploration for Droidal.

When enabled, the robot drives itself around unknown space to build the SLAM map:
it finds "frontiers" (boundaries between known-free and unknown cells in the live
`/map`), picks the best one, and sends it to Nav2 as a NavigateToPose goal.
Repeat until there are no frontiers left.

This only makes sense with slam_toolbox in **mapping** mode (SLAM_MODE=mapping),
where `/map` keeps growing. In localization mode the map is fixed and there are
no useful frontiers.

Safety: it is **opt-in** and starts DISABLED. Enable/disable at runtime:
    ros2 topic pub /explore/enable std_msgs/Bool "{data: true}"
    ros2 topic pub /explore/enable std_msgs/Bool "{data: false}"
Disabling cancels the active goal. Keep the drive power low while testing -- Nav2
avoids obstacles from `/scan`, but a mis-tuned base can still bump a wall.

No extra ROS packages needed: it reuses Nav2 (NavigateToPose) and numpy.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
import tf2_ros


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")

        # Tunables.
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("free_thresh", 25)         # <= this (and >=0) is free
        self.declare_parameter("occupied_thresh", 65)     # > this is an obstacle
        self.declare_parameter("cluster_size_m", 0.3)     # frontier bucket size
        self.declare_parameter("min_frontier_cells", 6)   # ignore tiny frontiers
        self.declare_parameter("gain_weight", 1.5)        # bias toward big frontiers
        self.declare_parameter("blacklist_radius_m", 0.5)
        self.declare_parameter("goal_timeout_s", 60.0)    # give up on a stuck goal
        self.declare_parameter("plan_period_s", 2.0)
        self.declare_parameter("start_enabled", False)

        g = self.get_parameter
        self.map_frame = g("map_frame").value
        self.base_frame = g("base_frame").value
        self.free_thresh = int(g("free_thresh").value)
        self.occ_thresh = int(g("occupied_thresh").value)
        self.cluster_size = float(g("cluster_size_m").value)
        self.min_cells = int(g("min_frontier_cells").value)
        self.gain_weight = float(g("gain_weight").value)
        self.blacklist_radius = float(g("blacklist_radius_m").value)
        self.goal_timeout = float(g("goal_timeout_s").value)

        self.enabled = bool(g("start_enabled").value)
        self.map = None
        self.goal_handle = None
        self.goal_active = False
        self.goal_xy = None
        self.goal_sent_time = None
        self.blacklist = []  # list of (x, y)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(Bool, "/explore/enable", self._on_enable, QoSProfile(depth=1))

        self.create_timer(float(g("plan_period_s").value), self._tick)
        self.get_logger().info(
            "explorer ready (DISABLED). Enable with: ros2 topic pub /explore/enable "
            "std_msgs/Bool \"{data: true}\" -- needs SLAM_MODE=mapping")

    # --- inputs --------------------------------------------------------------
    def _on_map(self, msg):
        self.map = msg

    def _on_enable(self, msg):
        if msg.data and not self.enabled:
            self.get_logger().info("exploration ENABLED")
        elif not msg.data and self.enabled:
            self.get_logger().info("exploration DISABLED")
            self._cancel_goal()
        self.enabled = bool(msg.data)

    # --- main loop -----------------------------------------------------------
    def _tick(self):
        if not self.enabled:
            return
        if self.goal_active:
            # Abort a goal that's taking too long, blacklist it, and move on.
            if self.goal_sent_time is not None:
                elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warning("goal timed out; blacklisting and reselecting")
                    if self.goal_xy is not None:
                        self.blacklist.append(self.goal_xy)
                    self._cancel_goal()
            return
        if self.map is None:
            return

        pose = self._robot_xy()
        if pose is None:
            self.get_logger().warning("no map->base_link TF yet", throttle_duration_sec=5.0)
            return

        target = self._pick_frontier(pose)
        if target is None:
            self.get_logger().info(
                "no reachable frontiers left -- exploration complete", throttle_duration_sec=10.0)
            return
        self._send_goal(target, pose)

    # --- frontier detection --------------------------------------------------
    def _pick_frontier(self, robot_xy):
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid <= self.free_thresh)
        unknown = grid < 0

        # A frontier is a free cell with an unknown 4-neighbour.
        unk_neighbour = np.zeros_like(unknown)
        unk_neighbour[1:, :] |= unknown[:-1, :]
        unk_neighbour[:-1, :] |= unknown[1:, :]
        unk_neighbour[:, 1:] |= unknown[:, :-1]
        unk_neighbour[:, :-1] |= unknown[:, 1:]
        frontier = free & unk_neighbour

        rows, cols = np.nonzero(frontier)
        if rows.size == 0:
            return None

        fx = ox + (cols + 0.5) * res
        fy = oy + (rows + 0.5) * res

        # Cluster frontier points into coarse buckets and take centroids.
        bx = np.floor(fx / self.cluster_size).astype(np.int64)
        by = np.floor(fy / self.cluster_size).astype(np.int64)
        buckets = {}
        for i in range(fx.size):
            key = (int(bx[i]), int(by[i]))
            acc = buckets.get(key)
            if acc is None:
                buckets[key] = [fx[i], fy[i], 1]
            else:
                acc[0] += fx[i]; acc[1] += fy[i]; acc[2] += 1

        rx, ry = robot_xy
        best = None
        best_cost = None
        for (sx, sy, n) in buckets.values():
            if n < self.min_cells:
                continue
            cx, cy = sx / n, sy / n
            if self._blacklisted(cx, cy):
                continue
            dist = math.hypot(cx - rx, cy - ry)
            # Lower is better: near frontiers, penalise, minus a size bonus.
            cost = dist - self.gain_weight * math.log(n + 1.0)
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best = (cx, cy)
        return best

    def _blacklisted(self, x, y):
        for (bx, by) in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    # --- Nav2 plumbing -------------------------------------------------------
    def _send_goal(self, target, robot_xy):
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("Nav2 not available yet", throttle_duration_sec=5.0)
            return
        cx, cy = target
        rx, ry = robot_xy
        yaw = math.atan2(cy - ry, cx - rx)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = cx
        goal.pose.pose.position.y = cy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_active = True
        self.goal_xy = (cx, cy)
        self.goal_sent_time = self.get_clock().now()
        self.get_logger().info(f"exploring -> x={cx:.2f} y={cy:.2f}")
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning("Nav2 rejected frontier goal; blacklisting")
            if self.goal_xy is not None:
                self.blacklist.append(self.goal_xy)
            self._clear_goal()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status  # 4=SUCCEEDED 5=CANCELED 6=ABORTED
        if status == 6 and self.goal_xy is not None:
            self.get_logger().info("frontier aborted; blacklisting")
            self.blacklist.append(self.goal_xy)
        self._clear_goal()

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._clear_goal()

    def _clear_goal(self):
        self.goal_active = False
        self.goal_handle = None
        self.goal_xy = None
        self.goal_sent_time = None

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return t.transform.translation.x, t.transform.translation.y


def main():
    rclpy.init()
    node = Explorer()
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
