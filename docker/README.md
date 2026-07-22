# Docker — ROS 2 Jazzy dev/test image

Tracking issue: **#4** (support ROS 2 Jazzy). This image builds and tests the package on ROS 2
Jazzy Jalisco (Ubuntu 24.04, PCL 1.14, Python 3.12) without touching the host, which runs Humble
natively. `colcon build` + `colcon test --packages-select batch_lio` are green inside the image.

## Why

`main` is verified on **ROS 2 Humble**. Jazzy ships newer rclcpp / PCL / Python; this image
isolates that compatibility work (see "Jazzy status" below).

## Build

```bash
docker build -t batch-lio:jazzy -f docker/jazzy.Dockerfile .
```

## Run (workspace mounted; edit on host, build in container)

```bash
REPO=/home/as/vllm/Batch-LIO                                        # this repo
LIVOXR2=/home/as/vllm/Nav/pb2025_sentry_nav/livox_ros_driver2       # livox_ros_driver2 source
BAG=/home/as/batch_lio_ws/bags/quick-shack                          # converted smoke bag (read-only)

docker run -it --rm \
  -v "$REPO":/ws/src/batch_lio \
  -v "$LIVOXR2":/ws/src/livox_ros_driver2 \
  -v "$BAG":/root/batch_lio_ws/bags/quick-shack:ro \
  -v batch-lio-jazzy-build:/ws/build \
  -v batch-lio-jazzy-install:/ws/install \
  -v batch-lio-jazzy-log:/ws/log \
  batch-lio:jazzy
```

The smoke launch_test (`test/test_smoke.py`) plays `~/batch_lio_ws/bags/quick-shack`, so mount the
host bag there (read-only is enough) for it to pass. The deskew gtest is standalone and needs no
bag.

Inside the container:

```bash
cd /ws
colcon build --symlink-install
colcon test --packages-select batch_lio
colcon test-result --all
```

The named volumes (`batch-lio-jazzy-{build,install,log}`) persist colcon artifacts across runs so
incremental rebuilds are fast; source is bind-mounted from the host.

## Jazzy status (issue #4)

Verified green: `100% tests passed, 0 failed` (deskew gtest: 5/5, smoke launch_test: 1/1).

What was checked and, where needed, fixed:

- **livox_ros_driver2 on Jazzy**: builds and links as-is. The vendored SDK2 `.so` is amd64 and links
  cleanly on Ubuntu 24.04; no version tweaks needed.
- **PCL 1.14 / rclcpp API drift**: the `src/` and `include/` code (including the type-tolerant
  param loader's `rclcpp::ParameterType::PARAMETER_*` form) compiles unchanged on PCL 1.14. Build
  stderr is limited to gcc-13/Eigen `-Wmaybe-uninitialized` false positives in `preprocess.cpp`.
- **rosbags output**: Jazzy's rosbag2 reads the Humble v5 `metadata.yaml` produced by
  `scripts/convert_bag.py` without modification (`ros2 bag info` / `bag play` both work).
- **Python 3.12 / PEP 668**: handled by `pip3 install --break-system-packages` in the Dockerfile.
- **Shutdown abort (fixed)**: on Jazzy/Fast-DDS the node aborted (SIGABRT) on Ctrl-C because the
  file-scope static `tf_broadcaster` was destroyed during `exit()`, after `rclcpp::shutdown()` had
  already torn the rcl context down. `src/laserMapping.cpp` now releases it before shutdown.
