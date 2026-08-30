#!/usr/bin/env python3
"""
Android command + telemetry bridge for Droidal.

Single transport, one node:

**WebSocket** (port ``~ws_port`` default 8791) -- all Android app traffic goes
here over a persistent TCP connection.  The ``process_request`` hook on the
same port also serves the static web visualiser (``GET /``, ``GET /app.js``,
``GET /style.css``) and the binary map PNG (``GET /map.png``) so browsers
continue to work without a separate HTTP port.

Messages from the phone arrive as JSON text frames:

  Commands (fire-and-forget; no reply required):
    {"type":"command","command":"explore","enable":true|false}
    {"type":"command","command":"freeze"}
    {"type":"command","command":"ping"}

  Requests (the phone includes an "id" and waits for a matching response):
    {"type":"request","id":"<uuid>","method":"GET","path":"/pose"}
    {"type":"request","id":"<uuid>","method":"GET","path":"/scan"}
    {"type":"request","id":"<uuid>","method":"GET","path":"/map.json"}
    {"type":"request","id":"<uuid>","method":"POST","path":"/goal",
     "body":{"x":1.0,"y":2.0,"yaw":0.0}}
    {"type":"request","id":"<uuid>","method":"POST","path":"/goal/cancel"}
    {"type":"request","id":"<uuid>","method":"POST","path":"/objects",
     "body":{"objects":[...]}}
    {"type":"request","id":"<uuid>","method":"GET","path":"/objects"}

Responses from the server:
    {"id":"<same>","result":<JSON value>}   -- success
    {"id":"<same>","error":"<message>"}     -- failure

Because the compose stack runs with ``network_mode: host`` the port is exposed
directly on the host, so the phone reaches it at ``<host-ip>:<port>``.

Params:
  ~ws_port        WebSocket (and HTTP viz) port to bind (default 8791).
  ~cmd_vel_topic  velocity topic to zero on freeze (default /cmd_vel).
  ~freeze_repeats how many zero-Twist messages to send on freeze (default 5).
  ~goal_topic     where to publish nav goals (default /move_base_simple/goal,
                  the topic goal_bridge listens on).
  ~map_frame / ~base_frame  TF frames for /pose (default map / base_link).
  ~objects_file   where pushed object landmarks are persisted (default
                  /opt/droidal/objects.json, on the bind-mounted mnt/).

Deps: std_msgs + geometry_msgs + nav_msgs + sensor_msgs + tf2_ros + numpy
(all already used elsewhere in the stack). websockets>=13 must be installed
in the Python environment (``pip install websockets``). PNG is encoded with
the stdlib (zlib) so no Pillow is required.
"""
import argparse
import base64
import http
import json
import math
import os
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler

import numpy as np
import rclpy
import websockets
import websockets.sync.server as ws_server
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


def _static_response(path):
    """Return (status, headers, body) for a static viz file, or None if not found."""
    rel = path.lstrip("/") or "index.html"
    abs_path = os.path.normpath(os.path.join(VIZ_DIR, rel))
    if not abs_path.startswith(VIZ_DIR) or not os.path.isfile(abs_path):
        return None
    ctype = ("text/html" if abs_path.endswith(".html")
             else "application/javascript" if abs_path.endswith(".js")
             else "text/css" if abs_path.endswith(".css")
             else "application/octet-stream")
    with open(abs_path, "rb") as f:
        body = f.read()
    return ctype, body


class AndroidBridge(Node):
    def __init__(self):
        super().__init__("android_bridge")

        self.declare_parameter("ws_port", 8791)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("freeze_repeats", 5)
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "objects_file",
            os.environ.get("OBJECTS_FILE", "/opt/droidal/objects.json"))

        self.ws_port = int(self.get_parameter("ws_port").value)
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

        # Telemetry inputs cached for WS request handlers. /map + /scan are
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

        # --- WebSocket server ------------------------------------------------
        self._ws_server = ws_server.serve(
            self._ws_handler,
            "0.0.0.0",
            self.ws_port,
            process_request=self._http_fallback,
        )
        self._ws_thread = threading.Thread(
            target=self._ws_server.serve_forever, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"android_bridge: WS 0.0.0.0:{self.ws_port} "
            f"(commands + RPC + viz), "
            f"{len(self.objects)} objects loaded from {self.objects_path}")

    # --- ROS telemetry callbacks --------------------------------------------
    def _on_map(self, msg):
        with self._lock:
            self._latest_map = msg

    def _on_scan(self, msg):
        with self._lock:
            self._latest_scan = msg

    # --- HTTP fallback for plain GET requests (viz + map.png) ---------------
    def _http_fallback(self, connection, request):
        """
        Called by the websockets sync server for every incoming HTTP request
        before the WS handshake. Return an HTTP response for non-upgrade
        requests (browser viz); return None to let the WS handshake proceed.
        """
        # WebSocket upgrade: let the library handle it normally.
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None

        path = request.path.split("?", 1)[0]

        if path == "/map.png":
            png = self.map_png()
            if png is None:
                body = b'{"error":"no map"}'
                return connection.respond(
                    http.HTTPStatus.SERVICE_UNAVAILABLE,
                    {"Content-Type": "application/json",
                     "Content-Length": str(len(body)),
                     "Access-Control-Allow-Origin": "*"},
                    body,
                )
            return connection.respond(
                http.HTTPStatus.OK,
                {"Content-Type": "image/png",
                 "Content-Length": str(len(png)),
                 "Access-Control-Allow-Origin": "*"},
                png,
            )

        # Static viz files (index.html, app.js, style.css, …)
        static = _static_response(path)
        if static is not None:
            ctype, body = static
            return connection.respond(
                http.HTTPStatus.OK,
                {"Content-Type": ctype,
                 "Content-Length": str(len(body)),
                 "Cache-Control": "no-cache"},
                body,
            )

        body = b'{"error":"not found"}'
        return connection.respond(
            http.HTTPStatus.NOT_FOUND,
            {"Content-Type": "application/json",
             "Content-Length": str(len(body))},
            body,
        )

    # --- WebSocket connection handler ----------------------------------------
    def _ws_handler(self, websocket):
        """Handle one WebSocket connection (runs in a dedicated thread per client)."""
        peer = websocket.remote_address
        self.get_logger().info(f"WS connect from {peer}")
        try:
            for raw in websocket:
                try:
                    msg = json.loads(raw)
                except (ValueError, UnicodeDecodeError):
                    self.get_logger().warning(
                        f"[{peer}] malformed WS frame: {raw[:64]!r}")
                    continue
                self._dispatch(websocket, msg, peer)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().info(f"WS {peer} closed: {exc}")

    def _dispatch(self, ws, msg, peer):
        """Route an incoming JSON message to the appropriate handler."""
        mtype = str(msg.get("type", "")).lower()

        if mtype == "command":
            self._handle_command(msg, peer)

        elif mtype == "request":
            mid = msg.get("id", "")
            try:
                result = self._handle_request(msg, peer)
                ws.send(json.dumps({"id": mid, "result": result}))
            except (KeyError, TypeError, ValueError) as exc:
                ws.send(json.dumps({"id": mid, "error": str(exc)}))

        else:
            self.get_logger().warning(f"[{peer}] unknown message type: {mtype!r}")

    # --- Command handler (fire-and-forget) -----------------------------------
    def _handle_command(self, msg, peer):
        command = str(msg.get("command", "")).lower()
        if command == "explore":
            enable = bool(msg.get("enable", False))
            self._explore_pub.publish(Bool(data=enable))
            self.get_logger().info(f"[{peer[0]}] explore -> {enable}")
        elif command in ("freeze", "stop"):
            self._freeze()
            self.get_logger().info(f"[{peer[0]}] freeze")
        elif command == "ping":
            self.get_logger().info(f"[{peer[0]}] ping")
        else:
            self.get_logger().warning(f"[{peer[0]}] unknown command: {msg!r}")

    def _freeze(self):
        # 1. Stop exploring so it doesn't immediately send a new goal.
        self._explore_pub.publish(Bool(data=False))
        # 2. Cancel any active Nav2 goal (goal_bridge/bt_navigator listen here).
        self._cancel_pub.publish(Empty())
        # 3. Belt-and-braces: command zero velocity a few times so the base
        #    halts even if a controller is mid-cycle.
        for _ in range(max(1, self.freeze_repeats)):
            self._cmd_vel_pub.publish(Twist())

    # --- Request handler (returns a JSON-serialisable value or raises) -------
    def _handle_request(self, msg, peer):
        method = str(msg.get("method", "GET")).upper()
        path = str(msg.get("path", ""))
        body = msg.get("body", {})

        if method == "GET":
            if path == "/pose":
                pose = self.current_pose()
                if pose is None:
                    raise ValueError("no pose available yet")
                return pose

            elif path == "/scan":
                scan = self.scan_snapshot()
                if scan is None:
                    raise ValueError("no scan available yet")
                return scan

            elif path == "/map.json":
                meta = self.map_metadata()
                if meta is None:
                    raise ValueError("no map available yet")
                return meta

            elif path == "/objects":
                return json.loads(self.objects_json())

            elif path.startswith("/thumb/"):
                oid = path[len("/thumb/"):]
                data = self.thumb_bytes(oid)
                if data is None:
                    raise ValueError(f"no thumbnail for {oid}")
                # Return base64 so it fits in a JSON text frame.
                return {"thumb_b64": base64.b64encode(data).decode("ascii")}

            else:
                raise ValueError(f"unknown GET path: {path}")

        elif method == "POST":
            if path == "/goal":
                x = float(body["x"])
                y = float(body["y"])
                yaw = float(body.get("yaw", 0.0))
                self.publish_goal(x, y, yaw)
                return {"result": "ok", "x": x, "y": y, "yaw": yaw}

            elif path == "/goal/cancel":
                self.cancel_goal()
                return {"result": "ok"}

            elif path == "/objects":
                records = body.get("objects") if isinstance(body, dict) else None
                if records is None:
                    records = [body] if isinstance(body, dict) else []
                stored = self.store_objects(records)
                return {"result": "ok", "stored": stored}

            else:
                raise ValueError(f"unknown POST path: {path}")

        else:
            raise ValueError(f"unsupported method: {method}")

    # --- Telemetry helpers (called from WS handler thread) -------------------
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

    # --- Object landmark store (for the visualiser) -------------------------
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
        try:
            self._ws_server.shutdown()
        except Exception:  # noqa: BLE001 -- best-effort on teardown
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ws-port", type=int, default=None,
                        help="WS port to bind (overrides the ~ws_port param).")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = AndroidBridge()
    if args.ws_port is not None and args.ws_port != node.ws_port:
        node.get_logger().info(
            f"(CLI --ws-port {args.ws_port} ignored; set the ~ws_port param instead)")
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
