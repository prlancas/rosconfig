# AGENT.md

Operational notes for AI agents working in this repo. Keep this current when
the architecture changes.

## What this project is

ROS 2 **Jazzy** config for the **Droidal** robot: a headless SLAM stack. An
ESP32 (micro-ROS) streams odometry + raw LIDAR; a host/container turns that into
a live `slam_toolbox` map. No graphical tools (no RViz) run here.

## Architecture (current)

Everything runs via `docker compose` (two containers):

1. `micro-ros-agent` — official `microros/micro-ros-agent:jazzy`, `udp4 --port 8888`.
2. `droidal` — custom image built from `Dockerfile`, runs `launch/bringup.launch.py`:
   - `mnt/droidal_viz.py` — URDF + `odom->base_link` + wheel/laser TFs from `/odrive_status`.
   - `mnt/lidar_scan_node.py` — raw Delta-2 bytes -> `/scan`.
   - `slam_toolbox` (async) — params from `mnt/my_slam_params.yaml`.
   - `mnt/goto_goal.py` — click-to-navigate: `/goal_pose` (PoseStamped, e.g. from
     Foxglove's Publish->Pose) -> `/cmd_vel`. Simple turn-then-drive controller,
     **no obstacle avoidance** (Nav2 is the planned upgrade). Conservative speeds;
     tunable via ROS params (`max_linear`, `goal_tolerance`, etc.). Reads robot
     pose from TF `map->base_link`. Note `max_linear/max_angular` are ODrive
     turns/s, not m/s (the ESP maps Twist onto ODrive velocity).
   - `foxglove_bridge` — websocket on `:8765` for headless visualization/teleop
     (connect the Foxglove app to `ws://<host>:8765`). Port overridable via
     `FOXGLOVE_PORT`. This is the debug/visualization surface; a custom product
     UI (semantic "go to the cooker" nav) will be a separate web app + nodes.

The image is `ros:jazzy-ros-base` (NOT `desktop-full`). `slam_toolbox` is baked
in at build time. `mnt/slam.sh` is legacy (it used to apt-install slam-toolbox at
runtime) and is no longer used.

### History / why
Previously: `run.sh` started two raw `docker run` containers and the user
`docker exec`'d into `desktop-full` and ran each script by hand in `/mnt`. This
was replaced by the compose + custom-image setup above.

## Data / hardware facts (don't relearn these)

- **LIDAR is a 3irobotics Delta-2.** Distance is encoded in **0.25 mm units**,
  so metres = `raw / 4000`. A bug previously used `raw / 1000` (every range 4x
  too far). See `_add_pkt` in `mnt/lidar_scan_node.py`.
- Packet framing: `0xAA` header, protocol `0x01`, type `0x61`, subtype `0xAD`
  (speed+measurements). 16 packets per 360° scan. Checksum = 16-bit cumulative
  sum of all bytes before the 2 checksum bytes. Angle scale 0.01°, speed `*0.05`.
- LIDAR source is chosen at runtime via env vars (`LIDAR_SERIAL`,
  `LIDAR_BAUD`, `LIDAR_TCP_PORT`, `LIDAR_EXTRA_ARGS`) read in the launch file.
- `droidal_viz.py` reads `/odrive_status` as space-separated `name:value`
  tokens; `parts[0]` = motor 0 (right), `parts[1]` = motor 1 (left). Wheel
  radius 0.095 m, wheel base 0.43 m. Right wheel feedback is sign-inverted.

## Common tasks

- **Run locally:** `./run.sh` (wraps `docker compose up`). `./run.sh down` to stop.
- **Live-edit scripts without rebuild:** uncomment the `./mnt:/opt/droidal`
  bind mount in `docker-compose.yml`.
- **Publish image:** push to `main` triggers `.github/workflows/docker-publish.yml`
  -> `ghcr.io/prlancas/droidalros`. Manual: `./push.sh` (after `docker login ghcr.io`).
- **Validate compose:** `docker compose config`.
- **SLAM mode** is env-selectable via `SLAM_MODE` in `docker-compose.yml`:
  `mapping` (build/extend, default in the launch) or `localization` (load the
  saved `mnt/droidal_map.*` read-only and relocalize — navigate without
  corrupting the map). `localization` uses `localization_slam_toolbox_node`.
- **Relocalize the robot** (after boot it starts at `map_start_pose` = wrong
  spot): in Foxglove use **Publish -> Pose estimate** (-> `/initialpose`,
  `PoseWithCovarianceStamped`), click the robot's real position and drag for
  heading. Driving in `mapping` mode from a wrong pose corrupts the map — that's
  what `localization` mode avoids.
- **Save the map:** `ros2 service call /slam_toolbox/serialize_map
  slam_toolbox/srv/SerializePoseGraph "{filename: /opt/droidal/droidal_map}"`,
  then `docker cp` the `.posegraph`/`.data` out to `mnt/` and rebuild.
- **Reset the SLAM map** (wipe and start mapping fresh, no restart needed):
  `ros2 service call /slam_toolbox/reset slam_toolbox/srv/Reset "{pause_new_measurements: false}"`.
- **Fix LIDAR orientation** (robot facing wrong way / map mirrored): tune
  `LIDAR_EXTRA_ARGS` in `docker-compose.yml`, e.g. `"--angle-offset 90"` (degrees,
  CCW) and/or `"--flip"` (mirror rotation). These are args to
  `lidar_scan_node.py` — no code edit needed. Recreate the container to apply
  (`./run.sh down && ./run.sh`). The physically-correct alternative is to set the
  `laser_joint` yaw in `mnt/droidal_viz.py` to match how the LIDAR is mounted.

## Gotchas (hard-won; don't re-debug)

- **`slam_toolbox` is a LIFECYCLE node on Jazzy.** Launched as a plain `Node` it
  boots *unconfigured* and silently does nothing — no `/scan` subscription, no
  `/map`, no `map->odom` TF. `bringup.launch.py` now launches it as a
  `LifecycleNode` and drives configure -> activate automatically. Symptom of
  regression: `ros2 topic info /scan` shows `Subscription count: 0`. Manual kick:
  `ros2 lifecycle set /slam_toolbox configure` then `... activate`.
- **Stale `ros2` daemon.** `ros2 topic list` showing only `/parameter_events`
  and `/rosout` while nodes are clearly publishing means the daemon cached an
  empty graph (it started before discovery settled). Fix: `ros2 daemon stop`
  (auto-restarts) or use `ros2 topic list --no-daemon`. Direct-discovery tools
  (`ros2 topic echo`, `foxglove_bridge`) are unaffected, which is the tell.
- **RViz on the Mac/host won't see topics** across the OrbStack/Docker DDS
  boundary (multicast is dropped). Use `foxglove_bridge` (WebSocket, no DDS) for
  visualization instead — that's why it's in the stack.
- **Restarting the stack orphans the ESP32.** `./run.sh down` destroys the
  `micro-ros-agent` container; the firmware connects to the agent once in
  `setup()` and does NOT auto-reconnect, so after any agent restart the ESP32
  stops publishing `/odrive_status`. Symptom: `ros2 topic info /odrive_status`
  shows `Publisher count: 0`, and slam_toolbox logs `Message Filter dropping
  message: frame 'laser_frame' ... queue is full` (it gets `/scan` over TCP but
  has no `odom->base_link->laser_frame` TF, which `droidal_viz.py` only
  publishes on `/odrive_status`). Fix: **power-cycle / reset the ESP32**. Proper
  fix: add micro-ROS reconnect logic to the firmware.

## Conventions

- Keep the image minimal — don't reintroduce desktop/GUI packages.
- Scripts in `mnt/` are baked into the image at `/opt/droidal` (env `DROIDAL_DIR`).
- The git remote uses SSH host alias `github-personal` (`prlancas/rosconfig`).
- Only commit when the user asks.
