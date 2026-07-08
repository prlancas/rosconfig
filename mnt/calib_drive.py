#!/usr/bin/env python3
"""TEMPORARY drive-calibration helper (safe to delete after calibration).

Commands a gentle, brief /cmd_vel and reports how /odom responds, so we can
objectively confirm the base obeys REP-103:
  +linear.x  -> robot moves forward, /odom d_forward > 0
  +angular.z -> robot turns LEFT (CCW),  /odom d_yaw > 0

Usage (inside the container):
  python3 calib_drive.py rot      # gentle spin, +angular.z (expect CCW / +yaw)
  python3 calib_drive.py rotneg   # gentle spin, -angular.z
  python3 calib_drive.py fwd      # gentle forward nudge (needs clear space!)
  python3 calib_drive.py back     # gentle reverse nudge

It always sends zero at the end; the ESP watchdog also stops the base.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calib(Node):
    def __init__(self):
        super().__init__("calib_drive")
        self.odom = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_odom(self, m):
        self.odom = m

    def pose(self, timeout=8.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                p = self.odom.pose.pose
                return p.position.x, p.position.y, _yaw(p.orientation)
        return None


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "rot"
    # Optional magnitude override (m/s for fwd/back, rad/s for rot/rotneg).
    mag = float(sys.argv[2]) if len(sys.argv) > 2 else None
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 1.2
    lin, ang = 0.0, 0.0
    if mode == "fwd":
        lin = mag if mag is not None else 0.15
    elif mode == "back":
        lin = -(mag if mag is not None else 0.15)
    elif mode == "rot":
        ang = mag if mag is not None else 1.0
    elif mode == "rotneg":
        ang = -(mag if mag is not None else 1.0)
    else:
        print(f"unknown mode '{mode}'"); return

    rclpy.init()
    n = Calib()
    start = n.pose()
    if start is None:
        print("NO /odom -- is droidal_viz running?"); n.destroy_node(); rclpy.shutdown(); return
    x0, y0, th0 = start

    cmd = Twist()
    cmd.linear.x = lin
    cmd.angular.z = ang
    end = time.time() + dur
    while time.time() < end:
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(1.0 / 15.0)
    stop = Twist()
    for _ in range(6):
        n.pub.publish(stop)
        rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.6)

    x1, y1, th1 = n.pose()
    dx, dy = x1 - x0, y1 - y0
    fwd = dx * math.cos(th0) + dy * math.sin(th0)
    dth = math.degrees(math.atan2(math.sin(th1 - th0), math.cos(th1 - th0)))

    print(f"\n=== RESULT mode={mode}  cmd(lin={lin:+.2f} ang={ang:+.2f}) ===")
    print(f"  d_forward = {fwd:+.3f} m   d_yaw = {dth:+.1f} deg   (dx={dx:+.3f} dy={dy:+.3f})")
    if mode == "fwd":
        print("  forward:", "PASS (+linear.x drives forward)" if fwd > 0.01
              else "FAIL -> flip BOTH axis signs in subscription_callback")
    elif mode == "back":
        print("  reverse:", "PASS" if fwd < -0.01 else "FAIL")
    elif mode == "rot":
        print("  +angular.z => CCW/+yaw:", "PASS" if dth > 2 else
              "FAIL -> flip ONE axis sign in subscription_callback (angular inverted)")
    elif mode == "rotneg":
        print("  -angular.z => CW/-yaw:", "PASS" if dth < -2 else "FAIL")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
