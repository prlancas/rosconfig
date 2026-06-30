"""Headless Droidal SLAM bringup.

Starts everything that used to be launched by hand inside the desktop-full
container:

  * droidal_viz.py        -> URDF, odom->base_link + wheel/laser TFs from /odrive_status
  * lidar_scan_node.py    -> raw Delta-2 bytes -> /scan
  * slam_toolbox          -> online async mapping (map->odom)
  * foxglove_bridge       -> websocket on :8765 so a browser/Foxglove app can
                             view /map, /scan, TF + robot model and teleop /cmd_vel

The lidar source is chosen from environment variables so the same image works
for the TCP forwarder or a USB-UART adapter without rebuilding:

  LIDAR_SERIAL      e.g. /dev/ttyUSB0  (if set, serial mode is used)
  LIDAR_BAUD        default 115200
  LIDAR_TCP_PORT    default 8080       (used when LIDAR_SERIAL is unset)
  LIDAR_EXTRA_ARGS  e.g. "--flip --angle-offset 90"
"""
import os
import shlex

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

PKG_DIR = os.environ.get("DROIDAL_DIR", "/opt/droidal")


def _lidar_cmd():
    cmd = ["python3", os.path.join(PKG_DIR, "lidar_scan_node.py")]
    serial_dev = os.environ.get("LIDAR_SERIAL")
    if serial_dev:
        cmd += ["--serial", serial_dev, "--baud", os.environ.get("LIDAR_BAUD", "115200")]
    else:
        cmd += ["--tcp", os.environ.get("LIDAR_TCP_PORT", "8080")]
    extra = os.environ.get("LIDAR_EXTRA_ARGS")
    if extra:
        cmd += shlex.split(extra)
    return cmd


def generate_launch_description():
    params_file = os.path.join(PKG_DIR, "my_slam_params.yaml")

    droidal_viz = ExecuteProcess(
        cmd=["python3", os.path.join(PKG_DIR, "droidal_viz.py")],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    lidar_node = ExecuteProcess(
        cmd=_lidar_cmd(),
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[params_file],
    )

    # Headless visualization/teleop surface. Connect with the Foxglove app
    # (desktop or app.foxglove.dev) to ws://<host>:8765 -- network_mode: host
    # means this port is exposed directly on the host. Port is overridable via
    # the FOXGLOVE_PORT env var.
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "address": "0.0.0.0",
            "port": int(os.environ.get("FOXGLOVE_PORT", "8765")),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([droidal_viz, lidar_node, slam, foxglove])
