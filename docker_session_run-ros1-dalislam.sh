#!/bin/bash
# Run DALI-SLAM (DA-LIO) on a rosbag (ROS 1 .bag or ROS 2 bag directory),
# record output topics, then convert the recorded bag to an HDMapping session.
#
# DA-LIO is FAST-LIO2-based: it publishes the registered world-frame scan on
# /cloud_registered and the body pose on /Odometry. We record both, then the
# converter chunks the world points and rebuilds the trajectory from /Odometry.

IMAGE_NAME='dalislam_noetic'
TMUX_SESSION='ros1_DALI-SLAM'

DATASET_CONTAINER_PATH='/ros_ws/dataset/input.bag'
CONVERTED_BAG_CONTAINER='/tmp/dataset_ros1.bag'
BAG_OUTPUT_CONTAINER='/ros_ws/recordings'

RECORDED_BAG_NAME="recorded-DALI-SLAM.bag"
HDMAPPING_OUT_NAME="output_hdmapping"

# Recorded topics (used by the converter).
ODOM_TOPIC="${ODOM_TOPIC:-/Odometry}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered}"

# Sensor selection: which DA-LIO config to load.
# helmet_mid | helmet_avia | avia | horizon | hesai | ouster64 | velodyne
SENSOR="${SENSOR:-helmet_mid}"

# RViz on by default — the live view of how DA-LIO tracks the dataset.
USE_RVIZ="${USE_RVIZ:-1}"

# Force Mesa software rendering by default so RViz renders even when the host
# GPU driver is not exposed to the container (the libGL "nvidia-drm" case).
LIBGL_SW="${LIBGL_SW:-1}"
if [[ "$LIBGL_SW" == "1" ]]; then LIBGL_ENV="1"; else LIBGL_ENV=""; fi

usage() {
  echo "Usage:"
  echo "  $0 <input.bag-or-ros2bag-dir> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  SENSOR        - DA-LIO config profile: helmet_mid|helmet_avia|avia|horizon|hesai|ouster64|velodyne (default: helmet_mid)"
  echo "  LIDAR_TOPIC   - LiDAR topic name inside the bag (default: the profile's config topic)"
  echo "  IMU_TOPIC     - IMU topic name inside the bag   (default: the profile's config topic)"
  echo "  ODOM_TOPIC    - DA-LIO odometry output topic    (default: /Odometry)"
  echo "  CLOUD_TOPIC   - DA-LIO registered cloud topic   (default: /cloud_registered)"
  echo "  USE_RVIZ      - 1/0, launch RViz live view      (default: 1)"
  exit 1
}

echo "=== DALI-SLAM (DA-LIO) rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file (or ROS 2 bag directory)")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
  exit 1
fi

if [[ ! -e "$DATASET_HOST_PATH" ]]; then
  echo "Error: input does not exist: $DATASET_HOST_PATH"
  exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"

DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

# Determine config file and the LiDAR/IMU topics it declares (so a bag using
# different names can be remapped on rosbag play).
case "$SENSOR" in
  helmet_mid)  CONFIG_FILE="helmet_mid.yaml";  CFG_LIDAR="/livox/lidar";        CFG_IMU="/imu0" ;;
  helmet_avia) CONFIG_FILE="helmet_avia.yaml"; CFG_LIDAR="/livox/lidar";        CFG_IMU="/imu0" ;;
  avia)        CONFIG_FILE="avia.yaml";        CFG_LIDAR="/livox/lidar";        CFG_IMU="/livox/imu" ;;
  horizon)     CONFIG_FILE="horizon.yaml";     CFG_LIDAR="/livox/lidar";        CFG_IMU="/livox/imu" ;;
  hesai)       CONFIG_FILE="hesai.yaml";       CFG_LIDAR="/hesai/pandar";       CFG_IMU="/alphasense/imu" ;;
  ouster64)    CONFIG_FILE="ouster64.yaml";    CFG_LIDAR="/os_cloud_node/points"; CFG_IMU="/os_cloud_node/imu" ;;
  velodyne)    CONFIG_FILE="velodyne.yaml";    CFG_LIDAR="/velodyne_points";    CFG_IMU="/imu/data" ;;
  *)
    echo "Error: unknown SENSOR=$SENSOR (use helmet_mid|helmet_avia|avia|horizon|hesai|ouster64|velodyne)"
    exit 1
    ;;
esac

# Bag-side topic names (default to the config topics → no remap needed)
LIDAR_TOPIC="${LIDAR_TOPIC:-$CFG_LIDAR}"
IMU_TOPIC="${IMU_TOPIC:-$CFG_IMU}"

# RViz on/off resolved to a roslaunch boolean
RVIZ_ARG=false; [[ "$USE_RVIZ" == "1" ]] && RVIZ_ARG=true

echo "Input           : $DATASET_HOST_PATH"
echo "Output dir      : $BAG_OUTPUT_HOST"
echo "Sensor profile  : $SENSOR  ($CONFIG_FILE)"
echo "LiDAR topic     : $LIDAR_TOPIC  (config expects $CFG_LIDAR)"
echo "IMU topic       : $IMU_TOPIC  (config expects $CFG_IMU)"
echo "Odom topic      : $ODOM_TOPIC"
echo "Cloud topic     : $CLOUD_TOPIC"

if [[ -d "$DATASET_HOST_PATH" ]]; then
  INPUT_IS_DIR=1
else
  INPUT_IS_DIR=0
fi

xhost +local:docker >/dev/null

# ── Phase 1: run DA-LIO + record output topics ───────────────────────────────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -e SENSOR="$SENSOR" \
  -e USE_RVIZ="$USE_RVIZ" \
  -e LIBGL_ALWAYS_SOFTWARE="$LIBGL_ENV" \
  -e CONFIG_FILE="$CONFIG_FILE" \
  -e CFG_LIDAR="$CFG_LIDAR" \
  -e CFG_IMU="$CFG_IMU" \
  -e LIDAR_TOPIC="$LIDAR_TOPIC" \
  -e IMU_TOPIC="$IMU_TOPIC" \
  -e ODOM_TOPIC="$ODOM_TOPIC" \
  -e CLOUD_TOPIC="$CLOUD_TOPIC" \
  -e INPUT_IS_DIR="$INPUT_IS_DIR" \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    source /opt/ros/noetic/setup.bash
    source /ros_ws/devel/setup.bash

    # ── If input is a ROS 2 bag directory, convert to a ROS 1 bag ──────────
    if [[ "$INPUT_IS_DIR" == "1" ]]; then
      echo "[convert] Converting ROS 2 bag to ROS 1 bag format..."
      rm -f '"$CONVERTED_BAG_CONTAINER"'
      rosbags-convert '"$DATASET_CONTAINER_PATH"' --dst '"$CONVERTED_BAG_CONTAINER"' || {
        echo "[convert] ERROR: rosbags-convert failed"; exit 1; }
      ROS1_BAG="'"$CONVERTED_BAG_CONTAINER"'"
    else
      ROS1_BAG="'"$DATASET_CONTAINER_PATH"'"
    fi

    export ROS1_BAG
    echo "[convert] ROS 1 bag ready at: $ROS1_BAG"
    ls -la $ROS1_BAG

    # Topic remap arguments for rosbag play, if the bag uses non-config names
    REMAP_ARGS=""
    if [[ "$LIDAR_TOPIC" != "$CFG_LIDAR" ]]; then
      REMAP_ARGS="$REMAP_ARGS $LIDAR_TOPIC:=$CFG_LIDAR"
    fi
    if [[ "$IMU_TOPIC" != "$CFG_IMU" ]]; then
      REMAP_ARGS="$REMAP_ARGS $IMU_TOPIC:=$CFG_IMU"
    fi
    export REMAP_ARGS
    echo "[play] rosbag remap args: $REMAP_ARGS"

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: roscore ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[roscore] starting..."
roscore
'\'' C-m

    # ---------- PANE 1: DA-LIO (+ RViz) ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 4
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
rosparam set /use_sim_time true
echo "[da_lio] launching run_dalio_bench.launch config_file='"$CONFIG_FILE"' (rviz='"$RVIZ_ARG"') ..."
roslaunch da_lio run_dalio_bench.launch config_file:='"$CONFIG_FILE"' rviz:='"$RVIZ_ARG"'
'\'' C-m

    # ---------- PANE 2: rosbag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 6
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
rm -f '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] start"
rosbag record '"$ODOM_TOPIC"' '"$CLOUD_TOPIC"' -O '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] exit"
'\'' C-m

    # ---------- PANE 3: rosbag play ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 10
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[play] start"
rosbag play --clock $ROS1_BAG $REMAP_ARGS; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- PANE 4: diagnostics ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 12
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "=== ROS 1 DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
rostopic list
echo ""
echo "--- Checking input LiDAR: '"$CFG_LIDAR"' ---"
timeout 5 rostopic hz '"$CFG_LIDAR"' 2>&1 &
echo ""
echo "--- Checking input IMU: '"$CFG_IMU"' ---"
timeout 5 rostopic hz '"$CFG_IMU"' 2>&1 &
echo ""
echo "--- Checking DA-LIO output: '"$ODOM_TOPIC"' ---"
timeout 5 rostopic hz '"$ODOM_TOPIC"' 2>&1 &
echo ""
echo "--- Checking DA-LIO output: '"$CLOUD_TOPIC"' ---"
timeout 5 rostopic hz '"$CLOUD_TOPIC"' 2>&1 &
wait
echo ""
echo "[diag] done — you can type ROS 1 commands here, e.g.:"
echo "  rostopic echo '"$ODOM_TOPIC"'"
'\'' C-m

    # ---------- Control window (window 1) ----------
    # This is the window the user attaches to; the 5 noisy panes are on window 0
    # (press Ctrl+b then 0 to watch RViz / SLAM output). It waits for the play
    # pane to signal end of playback, then tears the whole session down.
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[control] DA-LIO running — press Ctrl+b then 0 to watch RViz."
echo "[control] waiting for bag playback to finish..."
tmux wait-for BAG_DONE
echo "[control] bag playback finished — shutting down"

# Give DA-LIO a moment to process remaining queued scans
sleep 3

# Graceful stop: Ctrl+C to each pane
# Pane layout: 0=roscore, 1=da_lio+rviz, 2=recorder, 3=play, 4=diag
echo "[control] sending Ctrl+C to all panes..."
tmux send-keys -t '"$TMUX_SESSION"':0.2 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.1 C-c
sleep 1
tmux send-keys -t '"$TMUX_SESSION"':0.0 C-c
sleep 3

# Force-kill by process name
echo "[control] force-killing remaining processes..."
pkill -9 da_lio    2>/dev/null || true
pkill -9 rviz      2>/dev/null || true
pkill -9 rosmaster 2>/dev/null || true
pkill -9 rosout    2>/dev/null || true
sleep 1

echo "[control] terminating tmux"
tmux kill-server
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

# ── Phase 2: convert recorded bag to HDMapping session ────────────────────────
echo "=== Converting recorded bag to HDMapping session ==="

docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    source /ros_ws/devel/setup.bash
    rosrun dalislam_to_hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-DALI-SLAM\" \
      \"$ODOM_TOPIC\" \
      \"$CLOUD_TOPIC\"
  "

echo "=== DONE ==="
