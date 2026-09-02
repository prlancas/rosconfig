#!/usr/bin/env python3
"""
Android command + telemetry + exploration bridge for Droidal.

Single transport, one node:

**WebSocket** (port ``~ws_port`` default 8791) -- all Android app traffic goes
here over a persistent TCP connection. The ``process_request`` hook on the
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
    {"type":"request","id":"<uuid>","method":"GET","path":"/nav_status"}
    {"type":"request","id":"<uuid>","method":"GET","path":"/explore/targets"}
    {"type":"request","id":"<uuid>","method":"POST","path":"/goal",
     "body":{"x":1.0,"y":2.0,"yaw":0.0}}
    {"type":"request","id":"<uuid>","method":"POST","path":"/goal/cancel"}
    {"type":"request","id":"<uuid>","method":"POST","path":"/objects",
     "body":{"objects":[...]}}
    {"type":"request","id":"<uuid>","method":"GET","path":"/objects"}

Responses from the server:
    {"id":"<same>","result":<JSON value>}   -- success
    {"id":"<same>","error":"<message>"}     -- failure

Server push events:
    {"type":"event","event":"nav_status","status":"SUCCEEDED"|"ABORTED"|"CANCELED"|"NAVIGATING",
     "target":{"x":...,"y":...,"yaw":...}}

Because the compose stack runs with ``network_mode: host`` the port is exposed
directly on the host, so the phone reaches it at ``<host-ip>:<port>``.
"""
import argparse
import base64
import http
import json
import math
import os
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
import websockets
import websockets.sync.server as ws_server
from websockets.datastructures import Headers
from websockets.http11 import Response
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
import tf2_ros

VIZ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viz")


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _png_grayscale(width, height, pixels):
    """Encode a grayscale image (bytes-like, row-major, one byte/pixel) as PNG."""
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


def _http_response(status, headers, body):
    """Build an HTTP response for the non-WebSocket visualiser endpoints.

    ``ServerConnection.respond()`` only accepts text, which doesn't work for
    the PNG map and no longer accepts headers or a byte body in websockets 15.
    """
    status = http.HTTPStatus(status)
    return Response(status, status.phrase, Headers(headers), body)


class AndroidBridge(Node):
    def __init__(self):
        super().__init__("android_bridge")

        self.declare_parameter("ws_port", 8791)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("freeze_repeats", 5)
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("free_thresh", 25)
        self.declare_parameter("occupied_thresh", 65)
        self.declare_parameter(
            "objects_file",
            os.environ.get("OBJECTS_FILE", "/opt/droidal/objects.json"))

        self.ws_port = int(self.get_parameter("ws_port").value)
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.freeze_repeats = int(self.get_parameter("freeze_repeats").value)
        self.goal_topic = self.get_parameter("goal_topic").value
        action_name = self.get_parameter("action_name").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.free_thresh = int(self.get_parameter("free_thresh").value)
        self.occ_thresh = int(self.get_parameter("occupied_thresh").value)
        self.objects_path = self.get_parameter("objects_file").value

        self._explore_pub = self.create_publisher(Bool, "/explore/enable", QoSProfile(depth=1))
        self._cancel_pub = self.create_publisher(Empty, "/goal_pose/cancel", QoSProfile(depth=1))
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, QoSProfile(depth=1))
        self._goal_pub = self.create_publisher(PoseStamped, self.goal_topic, QoSProfile(depth=1))

        self._latest_scan = None
        self._latest_map = None
        self._lock = threading.Lock()

        # Nav2 action client for direct goal tracking and status reporting.
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._nav_goal_handle = None
        self._nav_status = "IDLE"  # IDLE, NAVIGATING, SUCCEEDED, ABORTED, CANCELED
        self._nav_target = None  # {x, y, yaw}
        self._nav_start_time = 0.0

        # Connected WebSocket clients for event broadcast.
        self._clients = set()
        self._clients_lock = threading.Lock()

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

    # --- ROS callbacks ------------------------------------------------------
    def _on_map(self, msg):
        with self._lock:
            self._latest_map = msg

    def _on_scan(self, msg):
        with self._lock:
            self._latest_scan = msg

    # --- HTTP fallback for plain GET requests (viz + map.png) ---------------
    def _http_fallback(self, connection, request):
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None

        path = request.path.split("?", 1)[0]

        if path == "/map.png":
            png = self.map_png()
            if png is None:
                body = b'{"error":"no map"}'
                return _http_response(
                    http.HTTPStatus.SERVICE_UNAVAILABLE,
                    {"Content-Type": "application/json",
                     "Content-Length": str(len(body)),
                     "Access-Control-Allow-Origin": "*"},
                    body,
                )
            return _http_response(
                http.HTTPStatus.OK,
                {"Content-Type": "image/png",
                 "Content-Length": str(len(png)),
                 "Access-Control-Allow-Origin": "*"},
                png,
            )

        static = _static_response(path)
        if static is not None:
            ctype, body = static
            return _http_response(
                http.HTTPStatus.OK,
                {"Content-Type": ctype,
                 "Content-Length": str(len(body)),
                 "Cache-Control": "no-cache"},
                body,
            )

        body = b'{"error":"not found"}'
        return _http_response(
            http.HTTPStatus.NOT_FOUND,
            {"Content-Type": "application/json",
             "Content-Length": str(len(body))},
            body,
        )

    # --- WebSocket connection handler ----------------------------------------
    def _ws_handler(self, websocket):
        peer = websocket.remote_address
        self.get_logger().info(f"WS connect from {peer}")
        with self._clients_lock:
            self._clients.add(websocket)
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
        finally:
            with self._clients_lock:
                self._clients.discard(websocket)

    def _broadcast_event(self, event_type, payload):
        frame = json.dumps({"type": "event", "event": event_type, **payload})
        with self._clients_lock:
            dead = set()
            for ws in self._clients:
                try:
                    ws.send(frame)
                except Exception:
                    dead.add(ws)
            self._clients.difference_update(dead)

    def _dispatch(self, ws, msg, peer):
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

    # --- Command handler ----------------------------------------------------
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
        self._explore_pub.publish(Bool(data=False))
        self.cancel_goal()
        for _ in range(max(1, self.freeze_repeats)):
            self._cmd_vel_pub.publish(Twist())

    # --- Request handler ----------------------------------------------------
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

            elif path == "/nav_status":
                return self.nav_status()

            elif path == "/explore/targets":
                return self.explore_targets()

            elif path == "/objects":
                return json.loads(self.objects_json())

            elif path.startswith("/thumb/"):
                oid = path[len("/thumb/"):]
                data = self.thumb_bytes(oid)
                if data is None:
                    raise ValueError(f"no thumbnail for {oid}")
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

    # --- Telemetry helpers ---------------------------------------------------
    def current_pose(self):
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
        with self._lock:
            s = self._latest_scan
        if s is None:
            return None
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
        with self._lock:
            m = self._latest_map
        if m is None:
            return None
        w, h = int(m.info.width), int(m.info.height)
        if w == 0 or h == 0:
            return None
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, dtype=np.uint8)
        img[(grid >= 0) & (grid <= 25)] = 254
        img[grid > 65] = 0
        img = np.flipud(img)
        return _png_grayscale(w, h, img.tobytes())

    # --- Navigation & Goal execution ----------------------------------------
    def nav_status(self):
        with self._lock:
            elapsed = (time.time() - self._nav_start_time) if self._nav_start_time > 0 else 0.0
            return {
                "status": self._nav_status,
                "target": self._nav_target,
                "elapsed_s": round(elapsed, 1),
            }

    def publish_goal(self, x, y, yaw=0.0):
        with self._lock:
            self._nav_status = "NAVIGATING"
            self._nav_target = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._nav_start_time = time.time()

        # 1. Publish to /move_base_simple/goal (for Foxglove/goal_bridge compatibility)
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._goal_pub.publish(goal)

        # 2. Also send directly via ActionClient if available for tracking
        if self._nav_client.wait_for_server(timeout_sec=0.5):
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal
            send_future = self._nav_client.send_goal_async(nav_goal)
            send_future.add_done_callback(self._on_goal_response)

        self._broadcast_event("nav_status", {"status": "NAVIGATING", "target": self._nav_target})
        self.get_logger().info(f"goal -> x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning("Nav2 rejected the goal")
            with self._lock:
                self._nav_status = "ABORTED"
            self._broadcast_event("nav_status", {"status": "ABORTED", "target": self._nav_target})
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        # status: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        status_code = future.result().status
        with self._lock:
            if status_code == 4:
                self._nav_status = "SUCCEEDED"
                self.get_logger().info(f"goal reached: {self._nav_target}")
            elif status_code == 5:
                self._nav_status = "CANCELED"
                self.get_logger().info("goal canceled")
            else:
                self._nav_status = "ABORTED"
                self.get_logger().warning(f"goal aborted (code {status_code})")
            target = self._nav_target
            self._nav_goal_handle = None

        self._broadcast_event("nav_status", {"status": self._nav_status, "target": target})

    def cancel_goal(self):
        self._cancel_pub.publish(Empty())
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        with self._lock:
            self._nav_status = "CANCELED"
            target = self._nav_target
        self._broadcast_event("nav_status", {"status": "CANCELED", "target": target})
        self.get_logger().info("goal cancel")

    # --- Exploration targets calculation (Frontiers + Wall Sampling) ---------
    def explore_targets(self):
        """
        Compute candidate exploration targets from live /map:
        1. Frontier clusters (boundaries between known-free and unknown space).
        2. Wall inspection vantage poses (facing wall segments).
        3. Doorway landmarks (from objects.json) to know room boundary portals.
        """
        with self._lock:
            m = self._latest_map
        if m is None:
            return {"frontiers": [], "wall_targets": [], "doors": [], "robot_pose": self.current_pose()}

        w, h = int(m.info.width), int(m.info.height)
        if w == 0 or h == 0:
            return {"frontiers": [], "wall_targets": [], "doors": [], "robot_pose": self.current_pose()}

        res = float(m.info.resolution)
        ox, oy = float(m.info.origin.position.x), float(m.info.origin.position.y)
        grid = np.asarray(m.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid <= self.free_thresh)
        unknown = grid < 0
        occupied = grid > self.occ_thresh

        # --- A. Frontiers (Free cells with unknown 4-neighbour) ---
        unk_neighbour = np.zeros_like(unknown)
        unk_neighbour[1:, :] |= unknown[:-1, :]
        unk_neighbour[:-1, :] |= unknown[1:, :]
        unk_neighbour[:, 1:] |= unknown[:, :-1]
        unk_neighbour[:, :-1] |= unknown[:, 1:]
        frontier = free & unk_neighbour

        rows, cols = np.nonzero(frontier)
        frontiers = []
        if rows.size > 0:
            fx = ox + (cols + 0.5) * res
            fy = oy + (rows + 0.5) * res
            cluster_size = 0.35  # meters
            bx = np.floor(fx / cluster_size).astype(np.int64)
            by = np.floor(fy / cluster_size).astype(np.int64)
            buckets = {}
            for i in range(fx.size):
                key = (int(bx[i]), int(by[i]))
                acc = buckets.get(key)
                if acc is None:
                    buckets[key] = [fx[i], fy[i], 1]
                else:
                    acc[0] += fx[i]; acc[1] += fy[i]; acc[2] += 1

            for (sx, sy, n) in buckets.values():
                if n >= 4:  # ignore tiny single-cell noise
                    cx, cy = sx / n, sy / n
                    frontiers.append({
                        "x": round(float(cx), 2),
                        "y": round(float(cy), 2),
                        "size": int(n),
                        "is_doorway": self._is_near_door(cx, cy),
                    })

        # --- B. Wall observation vantage points ---
        # Find occupied cells with free 4-neighbours (boundary walls).
        free_neighbour = np.zeros_like(free)
        free_neighbour[1:, :] |= free[:-1, :]
        free_neighbour[:-1, :] |= free[1:, :]
        free_neighbour[:, 1:] |= free[:, :-1]
        free_neighbour[:, :-1] |= free[:, 1:]
        wall_boundary = occupied & free_neighbour

        w_rows, w_cols = np.nonzero(wall_boundary)
        wall_targets = []
        if w_rows.size > 0:
            wx = ox + (w_cols + 0.5) * res
            wy = oy + (w_rows + 0.5) * res
            # Coarse sample of wall points every ~0.8m
            w_bucket_size = 0.8
            wbx = np.floor(wx / w_bucket_size).astype(np.int64)
            wby = np.floor(wy / w_bucket_size).astype(np.int64)
            w_buckets = {}
            for i in range(wx.size):
                key = (int(wbx[i]), int(wby[i]))
                if key not in w_buckets:
                    w_buckets[key] = (wx[i], wy[i])

            # For each sampled wall segment, find a vantage point ~0.7m into free space
            for (cx, cy) in w_buckets.values():
                vantage = self._find_free_vantage(grid, w, h, ox, oy, res, cx, cy, dist_m=0.7)
                if vantage is not None:
                    vx, vy, yaw = vantage
                    wall_targets.append({
                        "x": round(float(vx), 2),
                        "y": round(float(vy), 2),
                        "yaw": round(float(yaw), 2),
                        "wall_x": round(float(cx), 2),
                        "wall_y": round(float(cy), 2),
                    })

        # --- C. Doors ---
        doors = []
        with self._lock:
            for o in self.objects:
                if o.get("isDoor") and "worldX" in o and "worldY" in o:
                    doors.append({
                        "id": str(o.get("id", "")),
                        "canonical": str(o.get("canonical", "door")),
                        "label": str(o.get("label", "Door")),
                        "x": round(float(o["worldX"]), 2),
                        "y": round(float(o["worldY"]), 2),
                    })

        return {
            "robot_pose": self.current_pose(),
            "frontiers": frontiers,
            "wall_targets": wall_targets,
            "doors": doors,
        }

    def _find_free_vantage(self, grid, w, h, ox, oy, res, wx, wy, dist_m=0.7):
        """Find a free-space coordinate at dist_m from wall (wx, wy) facing towards the wall."""
        best_v = None
        for angle_deg in range(0, 360, 45):
            rad = math.radians(angle_deg)
            vx = wx + dist_m * math.cos(rad)
            vy = wy + dist_m * math.sin(rad)
            col = int(math.floor((vx - ox) / res))
            row = int(math.floor((vy - oy) / res))
            if 0 <= col < w and 0 <= row < h:
                val = grid[row, col]
                if 0 <= val <= self.free_thresh:
                    # Yaw points towards the wall: from vantage (vx, vy) to wall (wx, wy)
                    yaw = math.atan2(wy - vy, wx - vx)
                    best_v = (vx, vy, yaw)
                    break
        return best_v

    def _is_near_door(self, x, y, radius_m=1.2):
        with self._lock:
            for o in self.objects:
                if o.get("isDoor") and "worldX" in o and "worldY" in o:
                    dx = float(o["worldX"]) - x
                    dy = float(o["worldY"]) - y
                    if math.hypot(dx, dy) < radius_m:
                        return True
        return False

    # --- Object landmark store ----------------------------------------------
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
        except Exception:  # noqa: BLE001
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
