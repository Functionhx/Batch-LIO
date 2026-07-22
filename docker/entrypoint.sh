#!/bin/bash
# Container entrypoint: source ROS 2 Jazzy (and the workspace install if present).
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
