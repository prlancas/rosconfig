"""Headless Droidal SLAM bringup.

Starts everything that used to be launched by hand inside the desktop-full
container:

  * droidal_viz.py        -> URDF, odom->base_link + wheel/laser TFs from /odrive_status
  * lidar_scan_node.py    -> raw Delta-2 bytes -> /scan
  * slam_toolbox          -> online async mapping (map->odom)
  * goto_goal.py          -> /goal_pose (e.g. clicked in Foxglove) -> /cmd_vel
  * foxglove_bridge       -> websocket on :8765 so a browser/Foxglove app can
                             view /map, /scan, TF + robot model and teleop /cmd_vel

The lidar source is chosen from environment variables so the same image works
for the TCP forwarder or a USB-UART adapter without rebuilding:

  LIDAR_SERIAL      e.g. /dev/ttyUSB0  (if set, serial mode is used)
  LIDAR_BAUD        default 115200
  LIDAR_TCP_PORT    default 8080       (used when LIDAR_SERIAL is unset)
  LIDAR_EXTRA_ARGS  e.g. "--flip --angle-offset 90"

SLAM mode is also env-selectable:

  SLAM_MODE         "mapping" (default; build/extend the map) or "localization"
                    (load the saved map read-only and relocalize against it via
                    /initialpose -- use this to navigate without corrupting it).
"""
import os
import shlex

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

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

    # Click-to-navigate: turns a /goal_pose (from Foxglove's Publish->Pose tool)
    # into /cmd_vel. Simple controller, no obstacle avoidance (Nav2 comes later).
    goto_goal = ExecuteProcess(
        cmd=["python3", os.path.join(PKG_DIR, "goto_goal.py")],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    # SLAM mode (env-selectable): 'mapping' builds/extends the map; 'localization'
    # loads the saved map read-only and relocalizes against it (navigate without
    # corrupting the map). Each mode has its own executable.
    slam_mode = os.environ.get("SLAM_MODE", "mapping").strip().lower()
    slam_exe = ("localization_slam_toolbox_node" if slam_mode == "localization"
                else "async_slam_toolbox_node")

    # slam_toolbox is a LIFECYCLE node on Jazzy: launched as a plain Node it boots
    # "unconfigured" and never subscribes to /scan or publishes /map + map->odom.
    # Launch it as a LifecycleNode and drive it through configure -> activate, the
    # same thing slam_toolbox's own online_async_launch.py does with autostart.
    slam = LifecycleNode(
        package="slam_toolbox",
        executable=slam_exe,
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[params_file, {"use_lifecycle_manager": False, "mode": slam_mode}],
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
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

    return LaunchDescription([
        droidal_viz,
        lidar_node,
        goto_goal,
        foxglove,
        slam,
        slam_configure,
        slam_activate,
    ])
