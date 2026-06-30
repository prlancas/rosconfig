# rosconfig — Droidal headless SLAM stack

ROS 2 (Jazzy) configuration for the **Droidal** robot. It turns the robot's
ESP32 telemetry and a 3irobotics Delta-2 LIDAR into a live SLAM map, with no
graphical/desktop dependencies.

## What runs

Two containers, started together with `docker compose`:

| Container | Image | Job |
|-----------|-------|-----|
| `micro-ros-agent` | `microros/micro-ros-agent:jazzy` | Bridges the ESP32 (micro-ROS over UDP :8888) onto the DDS bus |
| `droidal` | `ghcr.io/prlancas/droidalros` (built here) | Runs all three nodes below + `slam_toolbox` |

Inside the `droidal` container a launch file starts:

- **`droidal_viz.py`** — publishes the URDF and `odom -> base_link` + wheel/laser
  TFs from `/odrive_status`.
- **`lidar_scan_node.py`** — parses raw Delta-2 bytes (TCP from the ESP32
  forwarder, or a USB-UART serial port) and publishes `/scan`.
- **`slam_toolbox`** — online async mapping using `mnt/my_slam_params.yaml`.
- **`foxglove_bridge`** — a websocket server (`:8765`) so you can view the live
  map, laser scan, TF and robot model — and teleop `/cmd_vel` — from the
  [Foxglove](https://foxglove.dev) app (desktop or web). Connect it to
  `ws://<host>:8765`. Override the port with `FOXGLOVE_PORT`.

The image is built from `ros:jazzy-ros-base`, **not** `desktop-full` — nothing
here is graphical, so the heavy RViz/GUI stack is omitted.

## Run it

```bash
./run.sh              # foreground (Ctrl-C to stop)
./run.sh -d           # background
./run.sh logs -f      # follow logs
./run.sh down         # stop & remove
```

`run.sh` is just a thin wrapper around `docker compose`. The first run builds
the image locally (or pulls it from GHCR if you've published it). To always
pull the published image instead of building, run `docker compose pull` first.

### Choosing the LIDAR source

Configured via environment variables in `docker-compose.yml` — no rebuild needed:

- **TCP (default):** listens for the ESP32 raw-lidar forwarder on `:8080`.
- **Serial:** set `LIDAR_SERIAL: "/dev/ttyUSB0"` (and optionally `LIDAR_BAUD`).
- **Orientation tuning:** `LIDAR_EXTRA_ARGS: "--flip --angle-offset 90"`.

## Publishing the image

CI builds and pushes a multi-arch image to **GHCR** on every push to `main`
(`.github/workflows/docker-publish.yml`) — no setup required, it uses the
built-in `GITHUB_TOKEN`. The package appears at
`ghcr.io/prlancas/droidalros`.

To publish manually from your machine:

```bash
echo "$GHCR_PAT" | docker login ghcr.io -u prlancas --password-stdin
./push.sh                 # ghcr.io/prlancas/droidalros:latest
TAG=v0.2 ./push.sh        # a specific tag
```

(`GHCR_PAT` = a GitHub Personal Access Token with the `write:packages` scope.)

## Repo layout

```
Dockerfile               Minimal headless ROS 2 image
docker-compose.yml       Orchestrates agent + droidal containers
entrypoint.sh            Sources ROS before exec
run.sh / push.sh         Convenience wrappers
launch/bringup.launch.py Starts all nodes + slam_toolbox
mnt/                     Robot scripts + SLAM params (baked into the image)
  droidal_viz.py
  lidar_scan_node.py
  my_slam_params.yaml
  slam.sh                (legacy; slam-toolbox is now baked into the image)
.github/workflows/       CI that publishes the image to GHCR
```
