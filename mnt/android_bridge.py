#!/usr/bin/env python3
"""
Android command bridge for Droidal: UDP JSON -> ROS 2 topics.

The Android app (Droidal's voice + face) can't speak DDS directly, so it sends
tiny JSON datagrams over UDP and this node translates them into the real ROS 2
topics. It's the "something listening" end of the app's `RobotBridge`
(droidal/Droidal/.../brain/tools/RobotBridge.kt).

Because the compose stack runs with `network_mode: host`, this UDP port is
exposed directly on the host, so the phone can reach it at `<host-ip>:<port>`
(or via subnet broadcast, which is the app's default so no IP needs
configuring).

Protocol (one JSON object per datagram):

  {"command": "explore", "enable": true}   -> /explore/enable  std_msgs/Bool(true)
  {"command": "explore", "enable": false}  -> /explore/enable  std_msgs/Bool(false)
  {"command": "freeze"}                     -> stop everything (see below)
  {"command": "ping"}                       -> logged only (health check)

"freeze" is the emergency stop the app sends when something is going wrong. It:
  * publishes /explore/enable false                 (stop frontier exploration)
  * publishes std_msgs/Empty on /goal_pose/cancel   (cancel the active Nav2 goal)
  * publishes a zero geometry_msgs/Twist on /cmd_vel a few times (halt the base)

All commands are idempotent, so the app safely re-sends safety-critical ones to
survive a dropped datagram.

Params:
  ~port           UDP port to bind (default 8790; must match the app's
                  DEFAULT_ROBOT_BRIDGE_PORT).
  ~cmd_vel_topic  velocity topic to zero on freeze (default /cmd_vel).
  ~freeze_repeats how many zero-Twist messages to send on freeze (default 5).

No extra ROS packages needed: std_msgs + geometry_msgs only.
"""
import argparse
import json
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist


class AndroidBridge(Node):
    def __init__(self):
        super().__init__("android_bridge")

        self.declare_parameter("port", 8790)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("freeze_repeats", 5)

        self.port = int(self.get_parameter("port").value)
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.freeze_repeats = int(self.get_parameter("freeze_repeats").value)

        # Latch explore/enable so a late-joining explorer still sees the last
        # command; the plain topics use a small depth-1 queue.
        self._explore_pub = self.create_publisher(Bool, "/explore/enable", QoSProfile(depth=1))
        self._cancel_pub = self.create_publisher(Empty, "/goal_pose/cancel", QoSProfile(depth=1))
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, QoSProfile(depth=1))

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Receive broadcast datagrams too (the app broadcasts by default).
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(("0.0.0.0", self.port))

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"android_bridge listening on udp/0.0.0.0:{self.port} "
            f"-> /explore/enable, /goal_pose/cancel, {cmd_vel_topic}")

    # --- UDP receive loop ----------------------------------------------------
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(4096)
            except OSError:
                break  # socket closed on shutdown
            try:
                msg = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                self.get_logger().warning(
                    f"ignoring malformed datagram from {addr}: {data[:64]!r}")
                continue
            self._handle(msg, addr)

    def _handle(self, msg, addr):
        command = str(msg.get("command", "")).lower()
        if command == "explore":
            enable = bool(msg.get("enable", False))
            self._explore_pub.publish(Bool(data=enable))
            self.get_logger().info(f"[{addr[0]}] explore -> {enable}")
        elif command in ("freeze", "stop"):
            self._freeze()
            self.get_logger().info(f"[{addr[0]}] freeze")
        elif command == "ping":
            self.get_logger().info(f"[{addr[0]}] ping")
        else:
            self.get_logger().warning(f"[{addr[0]}] unknown command: {msg!r}")

    def _freeze(self):
        # 1. Stop exploring so it doesn't immediately send a new goal.
        self._explore_pub.publish(Bool(data=False))
        # 2. Cancel any active Nav2 goal (goal_bridge/bt_navigator listen here).
        self._cancel_pub.publish(Empty())
        # 3. Belt-and-braces: command zero velocity a few times so the base
        #    halts even if a controller is mid-cycle.
        for _ in range(max(1, self.freeze_repeats)):
            self._cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=None,
                        help="UDP port to bind (overrides the ~port param).")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = AndroidBridge()
    if args.port is not None and args.port != node.port:
        node.get_logger().info(f"(CLI --port {args.port} ignored; set the ~port param instead)")
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
