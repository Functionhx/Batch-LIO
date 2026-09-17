# Batch-LIO dev/test image on ROS 2 Jazzy Jalisco (Ubuntu 24.04).
# Work in progress — see tracking issue #4 and docker/README.md.
#
# Build:
#   docker build -t batch-lio:jazzy -f docker/jazzy.Dockerfile .
# Run (mount the two source trees + named volumes for build/install):
#   docker run -it --rm \
#     -v $REPO:/ws/src/batch_lio \
#     -v $LIVOXR2:/ws/src/livox_ros_driver2 \
#     -v batch-lio-jazzy-build:/ws/build \
#     -v batch-lio-jazzy-install:/ws/install \
#     -v batch-lio-jazzy-log:/ws/log \
#     batch-lio:jazzy
# Inside the container:
#   cd /ws && colcon build --symlink-install
#   colcon test --packages-select batch_lio

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Build + runtime dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libpcl-dev libeigen3-dev libgoogle-glog-dev libomp-dev \
      python3-pip python3-colcon-common-extensions python3-vcstool git \
      ros-jazzy-pcl-conversions ros-jazzy-tf2-ros ros-jazzy-tf2-eigen \
      ros-jazzy-visualization-msgs ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# rosbags (ROS1->ROS2 bag conversion). Jazzy = Python 3.12, so PEP 668 applies.
RUN pip3 install --break-system-packages rosbags pyyaml

WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
