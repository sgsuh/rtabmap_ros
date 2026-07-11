# rtabmap_ros — ROS 2 Jazzy dev container

A self-contained Docker environment to **build and run this `rtabmap_ros` checkout on
ROS 2 Jazzy**. Nothing is installed on the host: ROS, the rtabmap core library and all
dependencies live inside the container. The repo source is bind-mounted, so you edit on
the host and build inside the container.

## Layout

| File | Purpose |
|------|---------|
| `Dockerfile` | Image based on `introlab3it/rtabmap:noble` (Jazzy + prebuilt rtabmap core), with this repo's rosdep deps installed. |
| `docker-compose.yml` | Bind-mounts the source, wires up NVIDIA GPU + WSLg X11, keeps the container alive. |

## Requirements (host)

- Docker Engine + Compose v2 (already present).
- For GPU: `nvidia-container-toolkit` (already configured for the RTX 4070).
  Remove the `gpus: all` line in `docker-compose.yml` to run CPU-only.
- For GUIs (rtabmap_viz / rviz2) under WSL2: WSLg provides `DISPLAY=:0` out of the box.

## Quick start

```bash
cd docker/jazzy/dev

# 1) Build the image (installs all ROS deps for the packages in this repo)
docker compose build

# 2) Start the container in the background
docker compose up -d

# 3) Open a shell inside it (ROS is auto-sourced)
docker compose exec dev bash
```

Inside the container:

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# sanity check
ros2 pkg list | grep rtabmap
```

The `build/`, `install/` and `log/` directories are stored in named Docker volumes,
so they survive `docker compose down` and are not written to the host tree.

## Running a GUI (rtabmap_viz / rviz2)

The image ships `rviz2` and the `image-transport-plugins` (needed to replay bags
recorded with `compressed`/`compressedDepth` images). GUIs render through WSLg's
X server (`DISPLAY=:0`).

```bash
# on the host, once per boot, allow local X clients (usually not needed under WSLg)
xhost +local: 2>/dev/null || true

# inside the container
rviz2                                 # or
ros2 run rtabmap_viz rtabmap_viz
```

### Example: robot_mapping demo with GUI

Mount your datasets by pointing `~/data` on the host at your bags (already wired in
`docker-compose.yml`). With `~/data/demo_mapping_bag` present:

```bash
# terminal 1 (inside the container): SLAM + both GUIs
ros2 launch rtabmap_demos robot_mapping_demo.launch.py rtabmap_viz:=true rviz:=true

# terminal 2 (inside the container): replay the dataset with the sim clock
docker compose exec dev bash
ros2 bag play /root/data/demo_mapping_bag --clock
```

The map database is written to `~/.ros/rtabmap.db` inside the container.

## Stop / clean up

```bash
docker compose down                 # stop container (keeps build volumes)
docker compose down -v              # also remove the build/install/log volumes
```
