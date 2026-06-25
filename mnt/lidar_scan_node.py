#!/usr/bin/env python3
"""
Host-side ROS 2 driver for the Delta-2 LIDAR.

Instead of decoding + publishing on the ESP32 (which can't keep up with reliable
DDS over WiFi), the ESP32 just forwards the raw UART bytes and this node — running
on a real machine — parses them and publishes sensor_msgs/LaserScan on /scan.

Data source (pick one):

  TCP (recommended): run the ESP32 `streamdata` firmware (raw UART -> TCP). It
  connects to <this-pc>:8080 and streams raw bytes. Start this node as a TCP
  *server* on the same port:
      python3 lidar_scan_node.py --tcp 8080
  (Point the firmware's SERVER_IP at this PC's IP, SERVER_PORT at 8080.)

  Serial: lidar TX -> a 3.3V USB-UART adapter on this PC:
      python3 lidar_scan_node.py --serial /dev/ttyUSB0 --baud 115200
      (requires:  pip install pyserial)

  Replay a raw capture file (for testing without hardware):
      python3 lidar_scan_node.py --file capture.dat

Requires a sourced ROS 2 environment (rclpy + sensor_msgs). Packet format and the
parser rationale are documented in LIDAR_PROTOCOL.md.
"""
import argparse
import math
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

STEP_DEG = 22.5 / 21.0
MIN_PKT = 17
MAX_PKT = 256


# --------------------------------------------------------------------------- #
#  Streaming, resyncing, checksum-validating parser (matches the firmware).
# --------------------------------------------------------------------------- #
class StreamParser:
    def __init__(self):
        self.buf = bytearray()
        self.valid = 0
        self.cksum_fail = 0
        self.discarded = 0

    def feed(self, data):
        b = self.buf
        b.extend(data)
        packets = []
        i, n, skip = 0, len(b), 0
        while True:
            if n - i < 6:
                break
            if not (b[i] == 0xAA and b[i + 1] == 0x00 and b[i + 3] == 0x01
                    and b[i + 4] == 0x61 and b[i + 5] == 0xAD):
                i += 1
                skip += 1
                continue
            total = ((b[i + 1] << 8) | b[i + 2]) + 2
            if total < MIN_PKT or total > MAX_PKT:
                i += 1
                skip += 1
                continue
            if n - i < total:
                break
            pkt = bytes(b[i:i + total])
            ck = (pkt[-2] << 8) | pkt[-1]
            if (sum(pkt[:-2]) & 0xFFFF) != ck:
                self.cksum_fail += 1
                i += 1
                skip += 1
                continue
            if skip:
                self.discarded += skip
                skip = 0
            self.valid += 1
            packets.append(pkt)
            i += total
        if skip:
            self.discarded += skip
        del b[:i]
        return packets


# --------------------------------------------------------------------------- #
class LidarScanNode(Node):
    def __init__(self, args):
        super().__init__("lidar_scan")
        self.args = args
        self.bins = args.bins
        self.parser = StreamParser()

        self.ranges = [math.inf] * self.bins
        self.intensities = [0.0] * self.bins
        self.last_angle = None
        self.last_rot = 0.0

        self.frames = 0
        self.lock = threading.Lock()
        self.increment = 2.0 * math.pi / self.bins

        qos = QoSProfile(depth=10)
        qos.reliability = (ReliabilityPolicy.BEST_EFFORT if args.best_effort
                           else ReliabilityPolicy.RELIABLE)
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(LaserScan, args.topic, qos)

        self.create_timer(5.0, self._log_stats)
        self.get_logger().info(
            f"publishing {args.topic} (frame_id='{args.frame_id}', "
            f"{self.bins} bins, {'best_effort' if args.best_effort else 'reliable'})")

    def _reset_scan(self):
        for i in range(self.bins):
            self.ranges[i] = math.inf
            self.intensities[i] = 0.0

    def feed(self, data):
        with self.lock:
            for pkt in self.parser.feed(data):
                self._add_pkt(pkt)

    def _add_pkt(self, pkt):
        start = ((pkt[11] << 8) | pkt[12]) / 100.0
        self.last_rot = pkt[8] * 0.05
        if self.last_angle is not None and start + 0.01 < self.last_angle:
            self._publish_scan()
            self._reset_scan()
        self.last_angle = start

        nsamp = (len(pkt) - 15) // 3
        for k in range(nsamp):
            o = 13 + k * 3
            d = (pkt[o + 1] << 8) | pkt[o + 2]
            if d == 0:
                continue
            r = d / 1000.0
            if r < self.args.range_min or r > self.args.range_max:
                continue
            ang = start + k * STEP_DEG
            if self.args.flip:
                ang = -ang
            ang = (ang + self.args.angle_offset) % 360.0
            idx = int(round(ang / 360.0 * self.bins)) % self.bins
            self.ranges[idx] = r
            self.intensities[idx] = float(pkt[o])

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - self.increment
        msg.angle_increment = self.increment
        scan_time = (1.0 / self.last_rot) if self.last_rot > 0.1 else 0.0
        msg.scan_time = scan_time
        msg.time_increment = scan_time / self.bins if self.bins else 0.0
        msg.range_min = float(self.args.range_min)
        msg.range_max = float(self.args.range_max)
        msg.ranges = list(self.ranges)
        msg.intensities = list(self.intensities)
        self.pub.publish(msg)
        self.frames += 1

    def _log_stats(self):
        with self.lock:
            self.get_logger().info(
                f"frames={self.frames} valid_pkts={self.parser.valid} "
                f"cksum_err={self.parser.cksum_fail} "
                f"discarded={self.parser.discarded} rot={self.last_rot:.2f} rev/s")


# --------------------------------------------------------------------------- #
#  Raw byte sources (run in a background thread, push into node.feed)
# --------------------------------------------------------------------------- #
def run_tcp(node, port, stop):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    srv.settimeout(1.0)
    node.get_logger().info(f"listening for raw lidar bytes on 0.0.0.0:{port}")
    while not stop.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        node.get_logger().info(f"forwarder connected: {addr}")
        conn.settimeout(1.0)
        try:
            while not stop.is_set():
                try:
                    data = conn.recv(8192)
                except socket.timeout:
                    continue
                if not data:
                    break
                node.feed(data)
        finally:
            conn.close()
            node.get_logger().info(f"forwarder disconnected: {addr}")


def run_serial(node, port, baud, stop):
    try:
        import serial
    except ImportError:
        node.get_logger().error("pyserial not installed. Run: pip install pyserial")
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"cannot open serial {port}: {e}")
        return
    node.get_logger().info(f"reading serial {port} @ {baud}")
    while not stop.is_set():
        data = ser.read(4096)
        if data:
            node.feed(data)


def run_file(node, path, baud, stop):
    try:
        data = open(path, "rb").read()
    except OSError as e:
        node.get_logger().error(f"cannot open {path}: {e}")
        return
    chunk = max(1, int(baud / 10 * 0.05))
    node.get_logger().info(f"replaying {path}: {len(data)} bytes (loop)")
    while not stop.is_set():
        for i in range(0, len(data), chunk):
            if stop.is_set():
                return
            node.feed(data[i:i + chunk])
            time.sleep(0.05)
        time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser(description="Delta-2 LIDAR -> ROS 2 /scan (host side)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--tcp", type=int, metavar="PORT",
                     help="listen for raw bytes from the ESP32 forwarder (e.g. 8080)")
    src.add_argument("--serial", metavar="DEV", help="read a serial/USB-UART port")
    src.add_argument("--file", metavar="PATH", help="replay a raw capture file")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--topic", default="scan")
    ap.add_argument("--frame-id", default="laser_frame")
    ap.add_argument("--bins", type=int, default=360, help="LaserScan resolution")
    ap.add_argument("--range-min", type=float, default=0.15)
    ap.add_argument("--range-max", type=float, default=10.0)
    ap.add_argument("--angle-offset", type=float, default=0.0,
                    help="degrees added to every angle (orientation tuning)")
    ap.add_argument("--flip", action="store_true",
                    help="reverse rotation direction (if the scan is mirrored)")
    ap.add_argument("--best-effort", action="store_true",
                    help="use BEST_EFFORT QoS instead of RELIABLE")
    args, ros_args = ap.parse_known_args()

    if not (args.tcp or args.serial or args.file):
        args.tcp = 8080  # sensible default: wait for the ESP32 forwarder

    rclpy.init(args=ros_args)
    node = LidarScanNode(args)

    stop = threading.Event()
    if args.tcp:
        target, sargs = run_tcp, (node, args.tcp, stop)
    elif args.serial:
        target, sargs = run_serial, (node, args.serial, args.baud, stop)
    else:
        target, sargs = run_file, (node, args.file, args.baud, stop)
    th = threading.Thread(target=target, args=sargs, daemon=True)
    th.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
