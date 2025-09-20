#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
if [ -f /root/ros1_ws/devel/setup.bash ]; then
  source /root/ros1_ws/devel/setup.bash
fi
exec "$@"
