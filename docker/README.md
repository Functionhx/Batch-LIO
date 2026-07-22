# Docker — ROS 2 Jazzy dev/test image

Tracking issue: **#4** (support ROS 2 Jazzy). This is a work-in-progress dev image so Jazzy
compatibility can be developed without touching the host (which runs Humble natively).

## Why

`main` is verified on **ROS 2 Humble** only. Jazzy Jalisco (Ubuntu 24.04, newer rclcpp/PCL/Python)
may need small fixes; this image isolates that work.

## Build

```bash
docker build -t batch-lio:jazzy -f docker/jazzy.Dockerfile .
```

## Run (workspace mounted; edit on host, build in container)

```bash
REPO=/home/as/vllm/Batch-LIO                                        # this repo
LIVOXR2=/home/as/vllm/Nav/pb2025_sentry_nav/livox_ros_driver2       # livox_ros_driver2 source

docker run -it --rm \
  -v "$REPO":/ws/src/batch_lio \
  -v "$LIVOXR2":/ws/src/livox_ros_driver2 \
  -v batch-lio-jazzy-build:/ws/build \
  -v batch-lio-jazzy-install:/ws/install \
  -v batch-lio-jazzy-log:/ws/log \
  batch-lio:jazzy
```

Inside the container:

```bash
cd /ws
colcon build --symlink-install
colcon test --packages-select batch_lio
colcon test-result --all
```

The named volumes (`batch-lio-jazzy-{build,install,log}`) persist colcon artifacts across runs so
incremental rebuilds are fast; source is bind-mounted from the host.

## Known Jazzy TODOs (see issue #4)

- **livox_ros_driver2 on Jazzy**: confirm its package/message build (SDK2 vendored `.so` is amd64;
  should link, but the driver package may need version tweaks).
- **PCL 1.14 / rclcpp API drift**: Humble used PCL 1.12; verify the `#include`s and the type-tolerant
  param loader still compile (e.g. `rclcpp::ParameterType` enum form).
- **rosbags output**: the converted-bag `metadata.yaml` was hand-shaped for Humble (v5); Jazzy's
  rosbag2 may want a different schema — re-check `scripts/convert_bag.py`.
- **Python 3.12 / PEP 668**: `pip install` needs `--break-system-packages` (already in the image) or a venv.
