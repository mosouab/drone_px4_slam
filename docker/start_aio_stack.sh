#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f /work/workspace/install/setup.bash ]; then
  source /work/workspace/install/setup.bash
fi

PX4_DIR="${PX4_AUTOPILOT_DIR:-/work/workspace/PX4-Autopilot}"
SESSION_NAME="drone-aio"

if [ ! -d "${PX4_DIR}" ]; then
  echo "[aio] PX4 introuvable: ${PX4_DIR}"
  echo "[aio] Lancez d'abord un shell du conteneur pour terminer le bootstrap."
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "[aio] Session tmux '${SESSION_NAME}' deja active."
  echo "[aio] Attachez-vous avec: tmux attach -t ${SESSION_NAME}"
  exit 0
fi

tmux new-session -d -s "${SESSION_NAME}" -n px4 \
  "cd ${PX4_DIR} && make px4_sitl gz_x500_depth"

tmux new-window -t "${SESSION_NAME}" -n mavros \
  "source /opt/ros/jazzy/setup.bash && source /work/workspace/install/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557"

tmux new-window -t "${SESSION_NAME}" -n clock \
  "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' --ros-args -p use_sim_time:=true"

tmux new-window -t "${SESSION_NAME}" -n camera \
  "source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image"

tmux new-window -t "${SESSION_NAME}" -n tf \
  "source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros static_transform_publisher 0.1 0 0 -1.5708 0 -1.5708 base_link camera_link"

tmux new-window -t "${SESSION_NAME}" -n rtabmap \
  "source /opt/ros/jazzy/setup.bash && ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_args:='--delete_db_on_start' rgb_topic:=/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image depth_topic:=/depth_camera camera_info_topic:=/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info frame_id:=base_link approx_sync:=true use_sim_time:=true"

tmux new-window -t "${SESSION_NAME}" -n relay \
  "source /opt/ros/jazzy/setup.bash && ros2 run topic_tools relay /rtabmap/odom /mavros/odometry/out"

tmux new-window -t "${SESSION_NAME}" -n mission \
  "source /opt/ros/jazzy/setup.bash && source /work/workspace/install/setup.bash && bash"

echo "[aio] Stack demarre dans tmux session: ${SESSION_NAME}"
echo "[aio] Attachez-vous avec: tmux attach -t ${SESSION_NAME}"
