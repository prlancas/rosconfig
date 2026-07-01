#!/usr/bin/env python3
"""
Labeled waypoint manager for Droidal.

Groundwork for "go to the cooker"-style commands from the Android app and from
VLM-tagged locations. It stores a map of {label: pose} on disk and lets you save
the robot's current spot under a name, or drive to a saved name.

Everything is plain std_msgs/String so it's trivial to drive from Foxglove, a
shell (`ros2 topic pub`), or the future Android app -- no custom messages.

Topics (subscribe):
  /waypoint/save   std_msgs/String
        "kitchen"                 -> save the robot's CURRENT map pose as "kitchen"
        "cooker,1.20,2.50,0.0"    -> save an explicit pose (x, y, yaw[rad]),
                                     e.g. a location a VLM tagged in the map
  /waypoint/goto   std_msgs/String
        "kitchen"                 -> publish a /goal_pose for that label
                                     (goal_bridge then drives there via Nav2)
  /waypoint/delete std_msgs/String
        "kitchen"                 -> forget that label

Topics (publish, latched):
  /waypoint/list   std_msgs/String   JSON {label: {x, y, yaw}} -- updated on change

Persistence: JSON at $WAYPOINTS_FILE (default /opt/droidal/waypoints.json), which
lives on the bind-mounted mnt/ so it survives container restarts.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter(
            "waypoints_file",
            os.environ.get("WAYPOINTS_FILE", "/opt/droidal/waypoints.json"))

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.path = self.get_parameter("waypoints_file").value

        self.waypoints = self._load()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 10)

        # Latched so a late subscriber (Android/Foxglove) still gets the list.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.list_pub = self.create_publisher(String, "/waypoint/list", latched)

        self.create_subscription(String, "/waypoint/save", self._on_save, QoSProfile(depth=5))
        self.create_subscription(String, "/waypoint/goto", self._on_goto, QoSProfile(depth=5))
        self.create_subscription(String, "/waypoint/delete", self._on_delete, QoSProfile(depth=5))

        self._publish_list()
        self.get_logger().info(
            f"waypoint_manager ready ({len(self.waypoints)} saved) file={self.path}")

    # --- persistence ---------------------------------------------------------
    def _load(self):
        try:
            with open(self.path) as f:
                data = json.load(f)
            if isinstance(data, dict):
                return data
        except FileNotFoundError:
            pass
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().warning(f"could not read {self.path}: {e}")
        return {}

    def _save_file(self):
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.waypoints, f, indent=2, sort_keys=True)
            os.replace(tmp, self.path)
        except OSError as e:
            self.get_logger().error(f"could not write {self.path}: {e}")

    def _publish_list(self):
        self.list_pub.publish(String(data=json.dumps(self.waypoints, sort_keys=True)))

    # --- current pose --------------------------------------------------------
    def _current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        return {"x": tr.x, "y": tr.y, "yaw": _yaw_from_quat(q.x, q.y, q.z, q.w)}

    # --- command handlers ----------------------------------------------------
    def _on_save(self, msg: String):
        parts = [p.strip() for p in msg.data.split(",")]
        label = parts[0]
        if not label:
            self.get_logger().warning("save: empty label ignored")
            return

        if len(parts) >= 3:
            try:
                x = float(parts[1])
                y = float(parts[2])
                yaw = float(parts[3]) if len(parts) >= 4 else 0.0
            except ValueError:
                self.get_logger().warning(f"save: bad explicit pose in '{msg.data}'")
                return
            pose = {"x": x, "y": y, "yaw": yaw}
        else:
            pose = self._current_pose()
            if pose is None:
                self.get_logger().warning(
                    "save: no map->base_link TF yet; can't record current pose",
                    throttle_duration_sec=5.0)
                return

        self.waypoints[label] = pose
        self._save_file()
        self._publish_list()
        self.get_logger().info(
            f"saved '{label}' at x={pose['x']:.2f} y={pose['y']:.2f} yaw={pose['yaw']:.2f}")

    def _on_goto(self, msg: String):
        label = msg.data.strip()
        pose = self.waypoints.get(label)
        if pose is None:
            self.get_logger().warning(
                f"goto: unknown label '{label}' (known: {list(self.waypoints)})")
            return
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(pose["x"])
        goal.pose.position.y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        self.get_logger().info(f"goto '{label}' -> published /goal_pose")

    def _on_delete(self, msg: String):
        label = msg.data.strip()
        if self.waypoints.pop(label, None) is not None:
            self._save_file()
            self._publish_list()
            self.get_logger().info(f"deleted '{label}'")
        else:
            self.get_logger().warning(f"delete: unknown label '{label}'")


def main():
    rclpy.init()
    node = WaypointManager()
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
