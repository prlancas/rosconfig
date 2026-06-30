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

## Conventions

- Keep the image minimal — don't reintroduce desktop/GUI packages.
- Scripts in `mnt/` are baked into the image at `/opt/droidal` (env `DROIDAL_DIR`).
- The git remote uses SSH host alias `github-personal` (`prlancas/rosconfig`).
- Only commit when the user asks.
