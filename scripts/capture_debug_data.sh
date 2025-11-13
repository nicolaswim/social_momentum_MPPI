#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

SESSION_ROOT=${SESSION_ROOT:-"$WS_ROOT/rosbags/debug_sessions"}
SESSION_NAME=${SESSION_NAME:-"session_$(date +%Y%m%d_%H%M%S)"}
SESSION_DIR="$SESSION_ROOT/$SESSION_NAME"
LOG_DIR="$SESSION_DIR/logs"
mkdir -p "$LOG_DIR"

RECORD_DURATION=${RECORD_DURATION:-600} # seconds
MONITOR_DURATION=${MONITOR_DURATION:-600}
BAG_TOPICS=${BAG_TOPICS:-"/tf /tf_static /clock /mobile_base_controller/odom /ground_truth_odom /model_states"}
TF_MONITOR_A_TARGET=${TF_MONITOR_A_TARGET:-odom}
TF_MONITOR_B_TARGET=${TF_MONITOR_B_TARGET:-base_footprint}

echo "Debug session directory: $SESSION_DIR"
echo "Recording topics: $BAG_TOPICS"
echo "Recording duration: ${RECORD_DURATION}s"

set +u
source /opt/ros/humble/setup.bash
if [ -f "$HOME/tiago_public_ws/install/setup.bash" ]; then
    source "$HOME/tiago_public_ws/install/setup.bash"
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
fi
set -u

# Snapshot current graph for post-mortem inspection
ros2 topic list >"$LOG_DIR/topic_list.txt"
ros2 node list >"$LOG_DIR/node_list.txt"

read -r -a BAG_ARGS <<<"$BAG_TOPICS"

has_tf2_monitor=false
if ros2 pkg executables tf2_ros >/dev/null 2>&1; then
    if ros2 pkg executables tf2_ros 2>/dev/null | grep -q "tf2_monitor"; then
        has_tf2_monitor=true
    fi
fi
MONITOR_A_LOG="$LOG_DIR/tf2_monitor_${TF_MONITOR_A_TARGET}_base_link.log"
MONITOR_B_LOG="$LOG_DIR/tf2_monitor_${TF_MONITOR_B_TARGET}_base_link.log"

if [ "$has_tf2_monitor" = false ]; then
    msg="Warning: tf2_ros/tf2_monitor not available. Skipping TF drift logs."
    echo "$msg"
    echo "$msg" >"$MONITOR_A_LOG"
    echo "$msg" >"$MONITOR_B_LOG"
fi

cleanup() {
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" >/dev/null 2>&1; then
            kill "$pid" >/dev/null 2>&1 || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT

PIDS=()

timeout "$RECORD_DURATION" ros2 bag record -o "$SESSION_DIR/bag" "${BAG_ARGS[@]}" \
    >"$LOG_DIR/rosbag_record.log" 2>&1 &
PIDS+=($!)

if [ "$has_tf2_monitor" = true ]; then
    timeout "$MONITOR_DURATION" ros2 run tf2_ros tf2_monitor "$TF_MONITOR_A_TARGET" base_link \
        >"$MONITOR_A_LOG" 2>&1 &
    PIDS+=($!)

    timeout "$MONITOR_DURATION" ros2 run tf2_ros tf2_monitor "$TF_MONITOR_B_TARGET" base_link \
        >"$MONITOR_B_LOG" 2>&1 &
    PIDS+=($!)
fi

wait "${PIDS[@]}"

echo "Debug capture complete. Data stored in $SESSION_DIR"
