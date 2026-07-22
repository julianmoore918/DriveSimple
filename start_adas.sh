#!/bin/bash
# start_adas.sh — launches the ADAS stack (ACC + LKAS).
#
# Assumes:
#   1. CARLA server is already running (./CarlaUE4.sh or 0.10 launcher)
#   2. carla-ros-bridge is already up and publishing /Car_1/* topics
#
# Brings up:
#   ACC :   perception_node      (YOLO lead-vehicle detection)
#           controller_node      (ACC throttle/brake + LKAS steer injection)
#   LKAS:   lane_detection_node  (UFLD V2 lane polylines)
#           stanley_node         (Stanley lateral controller)

# ── Check argument ──────────────────────────────────────
if [ "$1" != "carla" ] && [ "$1" != "morai" ]; then
    echo "Usage: ./start_adas.sh [carla|morai]"
    echo "  Note: LKAS nodes are CARLA-tuned. In morai mode they still launch"
    echo "        but lane detection / Stanley behaviour has not been validated."
    exit 1
fi

SIMULATOR=$1
echo "[INFO] Starting ADAS stack for: $SIMULATOR"

# ── Source ROS 2 & workspace ────────────────────────────
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

# ── Launch ACC nodes ────────────────────────────────────
ros2 run perception perception_node --ros-args -p simulator:=$SIMULATOR &
PERCEPTION_PID=$!
echo "[INFO] ACC perception node started (PID $PERCEPTION_PID)"

ros2 run controller controller_node --ros-args -p simulator:=$SIMULATOR &
CONTROLLER_PID=$!
echo "[INFO] ACC controller node started (PID $CONTROLLER_PID)"

# ── Launch LKAS nodes ───────────────────────────────────
ros2 run perception lane_detection_node --ros-args -p simulator:=$SIMULATOR &
LANE_DETECTION_PID=$!
echo "[INFO] LKAS lane_detection_node started (PID $LANE_DETECTION_PID)"

ros2 run controller stanley_node --ros-args -p simulator:=$SIMULATOR &
STANLEY_PID=$!
echo "[INFO] LKAS stanley_node started (PID $STANLEY_PID)"

# ── Launch debug-image fusion (combined ACC + LKAS view) ─
ros2 run perception debug_image_fusion_node &
FUSION_PID=$!
echo "[INFO] Debug-image fusion node started (PID $FUSION_PID)"

# ── Launch IPM bird's-eye view ──────────────────────────
ros2 run perception ipm_view_node &
IPM_VIEW_PID=$!
echo "[INFO] IPM view node started (PID $IPM_VIEW_PID)"

# ── MORAI adapter nodes ──────────────────────────────────
# Translate MORAI's ROS2 Interface topics (nav_msgs/Odometry,
# morai_v2_1_ros2_msgs/VehicleManualControl) to/from this stack's
# simulator-agnostic /Car_1/* topics. Not needed for CARLA, which has
# its own custom bridge (carlaaccsim) doing this job already.
if [ "$SIMULATOR" = "morai" ]; then
    ros2 run morai_bridge state_adapter_node &
    STATE_ADAPTER_PID=$!
    echo "[INFO] MORAI state adapter started (PID $STATE_ADAPTER_PID)"

    ros2 run morai_bridge control_adapter_node &
    CONTROL_ADAPTER_PID=$!
    echo "[INFO] MORAI control adapter started (PID $CONTROL_ADAPTER_PID)"
fi

# ── Shutdown handler ────────────────────────────────────
PIDS="$PERCEPTION_PID $CONTROLLER_PID $LANE_DETECTION_PID $STANLEY_PID $FUSION_PID $IPM_VIEW_PID $STATE_ADAPTER_PID $CONTROL_ADAPTER_PID"
trap "echo; echo '[INFO] Shutting down ADAS stack…'; kill $PIDS 2>/dev/null; exit 0" SIGINT SIGTERM

wait