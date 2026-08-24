#!/usr/bin/env python3
"""
Android command + telemetry bridge for Droidal.

Two transports, one node:

1. **UDP JSON** (unchanged, port ``~port`` default 8790) -- tiny, fire-and-forget
   safety commands from the app's ``RobotBridge.kt``. A dropped datagram must
   never leave the robot driving, so these are idempotent and re-sent by the app:

     {"command": "explore", "enable": true}   -> /explore/enable  std_msgs/Bool(true)
     {"command": "explore", "enable": false}  -> /explore/enable  std_msgs/Bool(false)
     {"command": "freeze"}                     -> stop everything (see _freeze)
     {"command": "ping"}                       -> logged only (health check)

2. **HTTP** (new, port ``~http_port`` default 8791) -- request/response for the
   richer spatial-memory link from the app's ``RobotHttpClient.kt``. UDP is a poor
   fit for reading state or uploading images, so those go over HTTP:

     GET  /pose         -> {"x","y","yaw","stamp"}         (TF map->base_link)
     GET  /scan         -> {"angle_min","angle_increment", (latest /scan)
                            "range_min","range_max","ranges":[...]}
     GET  /map.json     -> {"resolution","width","height","origin":{x,y,yaw}}
     GET  /map.png      -> occupancy grid as a grayscale PNG (north-up)
     POST /goal         {"x","y","yaw"?} -> PoseStamped on /move_base_simple/goal
                            (goal_bridge then drives there via Nav2)
     POST /goal/cancel  -> std_msgs/Empty on /goal_pose/cancel
     POST /objects      {"objects":[...]} | {single object} -> merge into
                            objects.json (+ optional base64 thumbnail per object)
     GET  /objects      -> the stored object landmarks (for the visualiser)
     GET  /thumb/<id>   -> the saved JPEG thumbnail for an object
     GET  / , /app.js   -> the static web visualiser (see viz/ alongside this file)

Because the compose stack runs with ``network_mode: host`` both ports are exposed
directly on the host, so the phone reaches them at ``<host-ip>:<port>`` (or, for
UDP only, via subnet broadcast which is the app's default).

Params:
  ~port           UDP port to bind (default 8790; matches DEFAULT_ROBOT_BRIDGE_PORT).
  ~http_port      HTTP port to bind (default 8791; matches DEFAULT_ROBOT_BRIDGE_HTTP_PORT).
  ~cmd_vel_topic  velocity topic to zero on freeze (default /cmd_vel).
  ~freeze_repeats how many zero-Twist messages to send on freeze (default 5).
  ~goal_topic     where to publish nav goals (default /move_base_simple/goal, the
                  topic goal_bridge listens on).
  ~map_frame / ~base_frame  TF frames for /pose (default map / base_link).
  ~objects_file   where pushed object landmarks are persisted (default
                  /opt/droidal/objects.json, on the bind-mounted mnt/).

Deps: std_msgs + geometry_msgs + nav_msgs + sensor_msgs + tf2_ros + numpy (all
already used elsewhere in the stack). PNG is encoded with the stdlib (zlib) so no
Pillow is required.
"""
import argparse
import base64
import json
import math
import os
import socket
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

# Directory holding the static web visualiser (index.html, app.js). Sits next to
# this script so it ships in the same bind-mounted mnt/ dir.
VIZ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viz")


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _png_grayscale(width, height, pixels):
    """Encode a grayscale image (bytes-like, row-major, one byte/pixel) as PNG.

    Pure stdlib (zlib) so the ROS image needs no Pillow. Color type 0 (gray),
    8-bit depth. Each scanline is prefixed with filter byte 0 (None).
    """
    def _chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff))

    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw.extend(pixels[y * width:(y + 1) * width])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + _chunk(b"IEND", b""))


class AndroidBridge(Node):
    def __init__(self):
        super().__init__("android_bridge")

        self.declare_parameter("port", 8790)
        self.declare_parameter("http_port", 8791)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("freeze_repeats", 5)
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "objects_file",
            os.environ.get("OBJECTS_FILE", "/opt/droidal/objects.json"))

        self.port = int(self.get_parameter("port").value)
        self.http_port = int(self.get_parameter("http_port").value)
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.freeze_repeats = int(self.get_parameter("freeze_repeats").value)
        self.goal_topic = self.get_parameter("goal_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.objects_path = self.get_parameter("objects_file").value

        # Latch explore/enable so a late-joining explorer still sees the last
        # command; the plain topics use a small depth-1 queue.
        self._explore_pub = self.create_publisher(Bool, "/explore/enable", QoSProfile(depth=1))
        self._cancel_pub = self.create_publisher(Empty, "/goal_pose/cancel", QoSProfile(depth=1))
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, QoSProfile(depth=1))
        self._goal_pub = self.create_publisher(PoseStamped, self.goal_topic, QoSProfile(depth=1))

        # Telemetry inputs cached for the HTTP GET handlers. /map + /scan are
        # sensor streams; we keep only the latest. TF is queried on demand.
        self._latest_scan = None
        self._latest_map = None
        self._lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, QoSProfile(depth=1))

        self.objects = self._load_objects()

        # --- UDP (unchanged) -------------------------------------------------
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(("0.0.0.0", self.port))

        self._stop = threading.Event()
        self._udp_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._udp_thread.start()

        # --- HTTP ------------------------------------------------------------
        self._http = ThreadingHTTPServer(("0.0.0.0", self.http_port), _make_handler(self))
        self._http_thread = threading.Thread(target=self._http.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f"android_bridge: UDP 0.0.0.0:{self.port} (explore/freeze) + "
            f"HTTP 0.0.0.0:{self.http_port} (pose/scan/map/goal/objects), "
            f"{len(self.objects)} objects loaded from {self.objects_path}")

    # --- ROS telemetry callbacks --------------------------------------------
    def _on_map(self, msg):
        with self._lock:
            self._latest_map = msg

    def _on_scan(self, msg):
        with self._lock:
            self._latest_scan = msg

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

    # --- HTTP-facing helpers (called from the HTTP server thread) ------------
    def current_pose(self):
        """TF map->base_link as {x, y, yaw, stamp}, or None if not available yet."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        stamp = t.header.stamp
        return {
            "x": tr.x, "y": tr.y,
            "yaw": _yaw_from_quat(q.x, q.y, q.z, q.w),
            "stamp": stamp.sec + stamp.nanosec * 1e-9,
        }

    def scan_snapshot(self):
        """Compact latest LaserScan, or None."""
        with self._lock:
            s = self._latest_scan
        if s is None:
            return None
        # Replace inf/nan with None so it survives JSON round-trip.
        ranges = [None if (r is None or math.isinf(r) or math.isnan(r)) else float(r)
                  for r in s.ranges]
        return {
            "angle_min": float(s.angle_min),
            "angle_max": float(s.angle_max),
            "angle_increment": float(s.angle_increment),
            "range_min": float(s.range_min),
            "range_max": float(s.range_max),
            "ranges": ranges,
        }

    def map_metadata(self):
        with self._lock:
            m = self._latest_map
        if m is None:
            return None
        o = m.info.origin
        return {
            "resolution": float(m.info.resolution),
            "width": int(m.info.width),
            "height": int(m.info.height),
            "origin": {
                "x": float(o.position.x),
                "y": float(o.position.y),
                "yaw": _yaw_from_quat(o.orientation.x, o.orientation.y,
                                      o.orientation.z, o.orientation.w),
            },
        }

    def map_png(self):
        """Render the latest occupancy grid as a north-up grayscale PNG, or None.

        Occupancy values: -1 unknown -> mid gray, 0 free -> white, 100 occupied
        -> black. OccupancyGrid row 0 is the origin (min y); PNG row 0 is the
        top, so rows are flipped vertically to render north-up.
        """
        with self._lock:
            m = self._latest_map
        if m is None:
            return None
        w, h = int(m.info.width), int(m.info.height)
        if w == 0 or h == 0:
            return None
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, dtype=np.uint8)          # unknown -> light gray
        img[(grid >= 0) & (grid <= 25)] = 254               # free -> white
        img[grid > 65] = 0                                  # occupied -> black
        img = np.flipud(img)                                # north-up
        return _png_grayscale(w, h, img.tobytes())

    def publish_goal(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._goal_pub.publish(goal)
        self.get_logger().info(f"goal -> x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def cancel_goal(self):
        self._cancel_pub.publish(Empty())
        self.get_logger().info("goal cancel")

    # --- object landmark store (for the visualiser) -------------------------
    def _load_objects(self):
        try:
            with open(self.objects_path) as f:
                data = json.load(f)
            if isinstance(data, list):
                return data
        except FileNotFoundError:
            pass
        except (json.JSONDecodeError, OSError) as e:
            self.get_logger().warning(f"could not read {self.objects_path}: {e}")
        return []

    def _save_objects(self):
        try:
            tmp = self.objects_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.objects, f, indent=2)
            os.replace(tmp, self.objects_path)
        except OSError as e:
            self.get_logger().error(f"could not write {self.objects_path}: {e}")

    def _thumb_dir(self):
        d = os.path.join(os.path.dirname(self.objects_path), "object_thumbs")
        os.makedirs(d, exist_ok=True)
        return d

    def store_objects(self, records):
        """Merge phone-pushed object records into the store (keyed by id).

        Each record: {id, canonical, label, aliases[], worldX, worldY,
        sourceX, sourceY, sourceYaw, confidence, isDoor, createdAt,
        thumbBase64?}. thumbBase64 (if present) is written to a JPEG on disk and
        replaced by a served "thumb" URL path so objects.json stays small.
        """
        stored = 0
        with self._lock:
            by_id = {o.get("id"): i for i, o in enumerate(self.objects) if o.get("id")}
            for rec in records:
                if not isinstance(rec, dict):
                    continue
                rec = dict(rec)
                oid = str(rec.get("id") or "").strip()
                thumb_b64 = rec.pop("thumbBase64", None)
                if oid and thumb_b64:
                    try:
                        raw = base64.b64decode(thumb_b64)
                        with open(os.path.join(self._thumb_dir(), f"{oid}.jpg"), "wb") as f:
                            f.write(raw)
                        rec["thumb"] = f"/thumb/{oid}"
                    except (ValueError, OSError) as e:
                        self.get_logger().warning(f"thumb write failed for {oid}: {e}")
                if oid and oid in by_id:
                    self.objects[by_id[oid]] = rec
                else:
                    self.objects.append(rec)
                    if oid:
                        by_id[oid] = len(self.objects) - 1
                stored += 1
            self._save_objects()
        return stored

    def objects_json(self):
        with self._lock:
            return json.dumps(self.objects)

    def thumb_bytes(self, oid):
        path = os.path.join(self._thumb_dir(), f"{oid}.jpg")
        try:
            with open(path, "rb") as f:
                return f.read()
        except OSError:
            return None

    def destroy_node(self):
        self._stop.set()
        try:
            self._http.shutdown()
        except Exception:  # noqa: BLE001 -- best-effort on teardown
            pass
        try:
            self._sock.close()
        except OSError:
            pass
        super().destroy_node()


def _make_handler(node):
    """Build a BaseHTTPRequestHandler bound to the given ROS node."""

    class Handler(BaseHTTPRequestHandler):
        # Silence the default noisy stderr logging; route to the ROS logger.
        def log_message(self, fmt, *args):
            node.get_logger().debug("http: " + (fmt % args))

        def _send_json(self, obj, code=200):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, data, content_type, code=200):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)

        def _read_json_body(self):
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            try:
                return json.loads(raw.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                return None

        def _serve_static(self, rel):
            path = os.path.normpath(os.path.join(VIZ_DIR, rel))
            if not path.startswith(VIZ_DIR) or not os.path.isfile(path):
                self._send_json({"error": "not found"}, 404)
                return
            ctype = ("text/html" if path.endswith(".html")
                     else "application/javascript" if path.endswith(".js")
                     else "text/css" if path.endswith(".css")
                     else "application/octet-stream")
            with open(path, "rb") as f:
                self._send_bytes(f.read(), ctype)

        def do_GET(self):  # noqa: N802 -- BaseHTTPRequestHandler API
            route = self.path.split("?", 1)[0]
            if route == "/pose":
                pose = node.current_pose()
                self._send_json(pose or {"error": "no pose"}, 200 if pose else 503)
            elif route == "/scan":
                scan = node.scan_snapshot()
                self._send_json(scan or {"error": "no scan"}, 200 if scan else 503)
            elif route == "/map.json":
                meta = node.map_metadata()
                self._send_json(meta or {"error": "no map"}, 200 if meta else 503)
            elif route == "/map.png":
                png = node.map_png()
                if png is None:
                    self._send_json({"error": "no map"}, 503)
                else:
                    self._send_bytes(png, "image/png")
            elif route == "/objects":
                self._send_bytes(node.objects_json().encode("utf-8"), "application/json")
            elif route.startswith("/thumb/"):
                oid = route[len("/thumb/"):]
                data = node.thumb_bytes(oid)
                if data is None:
                    self._send_json({"error": "not found"}, 404)
                else:
                    self._send_bytes(data, "image/jpeg")
            elif route in ("/", "/index.html"):
                self._serve_static("index.html")
            elif route in ("/app.js", "/style.css"):
                self._serve_static(route.lstrip("/"))
            else:
                self._send_json({"error": "not found"}, 404)

        def do_POST(self):  # noqa: N802 -- BaseHTTPRequestHandler API
            route = self.path.split("?", 1)[0]
            body = self._read_json_body()
            if body is None:
                self._send_json({"error": "bad json"}, 400)
                return
            if route == "/goal":
                try:
                    x = float(body["x"])
                    y = float(body["y"])
                    yaw = float(body.get("yaw", 0.0))
                except (KeyError, TypeError, ValueError):
                    self._send_json({"error": "need numeric x,y[,yaw]"}, 400)
                    return
                node.publish_goal(x, y, yaw)
                self._send_json({"result": "ok", "x": x, "y": y, "yaw": yaw})
            elif route == "/goal/cancel":
                node.cancel_goal()
                self._send_json({"result": "ok"})
            elif route == "/objects":
                records = body.get("objects") if isinstance(body, dict) else None
                if records is None:
                    records = [body] if isinstance(body, dict) else []
                stored = node.store_objects(records)
                self._send_json({"result": "ok", "stored": stored})
            else:
                self._send_json({"error": "not found"}, 404)

    return Handler


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
