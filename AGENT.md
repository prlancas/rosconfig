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
   - `mnt/droidal_viz.py` — URDF + `/odom` (nav_msgs/Odometry) + `odom->base_link`
     + wheel/laser TFs from `/odrive_status`. Also republishes the ESP's `V:`
     token as `/battery_voltage` (Float32). The lidar's mounting orientation is
     baked into the `laser_frame` TF here (no scan mutation).
   - `mnt/lidar_scan_node.py` — raw Delta-2 bytes -> `/scan` (published as-is).
   - `slam_toolbox` (async/localization) — params from `mnt/my_slam_params.yaml`.
   - **Nav2** — `controller_server` (RegulatedPurePursuitController) +
     `planner_server` (Navfn) + local/global costmaps (from `/scan`) +
     `behavior_server` + `bt_navigator` + `velocity_smoother`, brought up by
     `nav2_lifecycle_manager` (autostart). Params in `mnt/nav2_params.yaml`.
     Consumes `NavigateToPose` goals, outputs plain `geometry_msgs/Twist` on
     `/cmd_vel`. No amcl/map_server — `slam_toolbox` localization gives `map->odom`.
   - `mnt/goal_bridge.py` — click-to-navigate front door: `/goal_pose`
     (PoseStamped, e.g. Foxglove Publish->Pose, or the future Android app; any
     frame -> transformed to `map`) -> Nav2 `NavigateToPose` action. Cancel via
     `std_msgs/Empty` on `/goal_pose/cancel`.
   - `mnt/waypoint_manager.py` — labeled targets. `/waypoint/save`,
     `/waypoint/goto`, `/waypoint/delete` (all `std_msgs/String`); persists
     `{label: {x,y,yaw}}` to `mnt/waypoints.json`; `goto` republishes `/goal_pose`.
     This is the groundwork for "go to the cooker" (Android app + VLM-tagged spots).
   - `mnt/explorer.py` — opt-in autonomous frontier exploration. Finds `/map`
     frontiers (numpy) and sends them to Nav2 (`NavigateToPose`) to self-map;
     blacklists unreachable/timed-out frontiers. Starts DISABLED; toggle with
     `/explore/enable` (`std_msgs/Bool`). Only useful with `SLAM_MODE=mapping`.
   - `mnt/android_bridge.py` — UDP JSON command bridge for the Android app
     (which can't speak DDS). Binds `udp/0.0.0.0:8790` (`~port` param; exposed
     directly via `network_mode: host`) and republishes to ROS topics:
     `{"command":"explore","enable":bool}` -> `/explore/enable`;
     `{"command":"freeze"}` -> `/explore/enable false` + `std_msgs/Empty` on
     `/goal_pose/cancel` + a zero `geometry_msgs/Twist` on `/cmd_vel`. The app
     end is `RobotBridge.kt` (default: subnet broadcast, so no host IP needed).
   - `foxglove_bridge` — websocket on `:8765` for headless visualization/teleop
     (connect the Foxglove app to `ws://<host>:8765`). Port overridable via
     `FOXGLOVE_PORT`. This is the debug/visualization surface; a custom product
     UI (semantic "go to the cooker" nav) will be a separate web app + nodes.
   - `mnt/goto_goal.py` — **retired** (open-loop turn-then-drive, no obstacle
     avoidance, needed an `angular_sign=-1` hack). Left in the tree for reference
     but no longer launched; Nav2 + `goal_bridge.py` replace it.

The image is `ros:jazzy-ros-base` (NOT `desktop-full`). `slam_toolbox` and Nav2
(`navigation2` + `nav2-bringup`) are baked in at build time. `mnt/slam.sh` is
legacy (it used to apt-install slam-toolbox at runtime) and is no longer used.

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
  Odometry is metric (m) and REP-103 compliant: forward -> `+x`, CCW -> `+yaw`.

### Base contract (REP-103) — keep firmware and odometry in sync
- **`/cmd_vel` is SI**: `linear.x` in m/s (+ = forward), `angular.z` in rad/s
  (+ = CCW / turn left). The **ESP32 firmware** does the diff-drive mixing and
  converts m/s -> ODrive wheel turns/s (using the same 0.095 m radius / 0.43 m
  base), so ROS-side everything is metric and Nav2 needs **no sign hacks**.
- If you change wheel geometry, update it in **both** `droidalesp/src/main.cpp`
  (the `WHEEL_RADIUS_M`/`WHEEL_BASE_M` constants in `subscription_callback`) and
  `droidal_viz.py`, or odometry and commands diverge.
- Motor/axis mapping lives in the firmware: axis 1 = LEFT (forward = +cmd),
  axis 0 = RIGHT (mirrored, forward = -cmd). Flip a single axis's sign there if a
  direction comes out reversed after reflashing.

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
- **Navigate (click-to-go):** in Foxglove use **Publish -> Pose** on `/goal_pose`
  (any display frame works — `goal_bridge.py` transforms it to `map`). It becomes
  a Nav2 `NavigateToPose` goal; watch the planned path + costmaps in Foxglove.
  Cancel: publish `std_msgs/Empty` on `/goal_pose/cancel`.
- **Labeled waypoints:** `ros2 topic pub --once /waypoint/save std_msgs/String
  "{data: 'kitchen'}"` saves the current pose; `.../waypoint/goto` "{data:
  'kitchen'}" drives there. Explicit pose (e.g. VLM-tagged):
  `"{data: 'cooker,1.2,2.5,0.0'}"` (label,x,y,yaw). Stored in `mnt/waypoints.json`.
- **Auto-explore (self-map):** set `SLAM_MODE: "mapping"`, then
  `ros2 topic pub --once /explore/enable std_msgs/Bool "{data: true}"`. The robot
  drives Nav2 toward `/map` frontiers until none remain. `"{data: false}"` stops
  it (cancels the active goal). Keep drive power low while testing. Save the
  result with `slam_toolbox/serialize_map` (below), then switch back to
  `localization`. Nav2 + mapping run together, so the map grows as it navigates —
  that's why localization mode "can't map while navigating" (its map is fixed).
- **Battery voltage:** `ros2 topic echo /battery_voltage` (Float32), or the web
  UI's Battery readout. Firmware voltage-compensates the gains, so the base
  shouldn't need the power slider turned down when fully charged anymore.
- **"Map looks rotated":** the `map` frame's orientation is set by the robot's
  heading when mapping starts, so a fresh map can look rotated vs an old one —
  that's cosmetic. What matters is that walls stay crisp (don't smear) as the
  robot turns; if they smear, calibrate the `laser_frame` yaw (see below). After
  the firmware/lidar-TF changes, **rebuild the map** — an old saved map from the
  previous convention may not line up.
- **Fix LIDAR orientation** (robot facing wrong way / map mirrored): the mounting
  orientation is baked into the `laser_frame` TF in `mnt/droidal_viz.py` (the
  `t_laser` rotation quaternion, currently roll=pi + yaw=pi/2 to match the
  flipped, 90°-rotated Delta-2). Adjust that quaternion (and the matching
  `laser_joint` rpy in the URDF) rather than mutating the raw scan. Calibrate by
  facing a wall so `/scan` shows it straight ahead (`+x`). The old
  `LIDAR_EXTRA_ARGS: "--flip --angle-offset 90"` hack is removed; a single
  `--angle-offset` can still be added for fine tuning.

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
