#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-/work/workspace/PX4-Autopilot}"
PX4_GIT_REF="${PX4_GIT_REF:-v1.16.0}"

if [ ! -d "${PX4_AUTOPILOT_DIR}" ]; then
  echo "[entrypoint] Clonage PX4-Autopilot (${PX4_GIT_REF})..."
  git clone --recursive --branch "${PX4_GIT_REF}" https://github.com/PX4/PX4-Autopilot.git "${PX4_AUTOPILOT_DIR}"
fi

if [ -d /work/workspace/src/ros_src ] && [ ! -f /work/workspace/install/setup.bash ]; then
  echo "[entrypoint] Premier lancement: build du workspace ROS..."
  cd /work/workspace
  colcon build --symlink-install --base-paths src/ros_src
fi

if [ -f /work/workspace/install/setup.bash ]; then
  source /work/workspace/install/setup.bash
fi

exec "$@"
