#!/usr/bin/env bash

set -euo pipefail

# How long to let each simulation run (in seconds)
RUN_DURATION=${RUN_DURATION:-150} # 2.5 minutes
COOLDOWN=${COOLDOWN:-10}

COMMAND=("$@")
if [ ${#COMMAND[@]} -eq 0 ]; then
    COMMAND=(./rebuild_and_launch.sh)
fi

cleanup() {
    echo
    echo "Stopping automated simulation batch run."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Starting automated simulation batch run..."
echo "Command: ${COMMAND[*]}"
echo "Each run will last $RUN_DURATION seconds (2.5 minutes)."
echo "Press Ctrl+C in this terminal to stop the loop."

while true; do
    echo "-----------------------------------"
    echo "Starting new simulation run at $(date -Iseconds)..."

    if timeout --preserve-status --signal=SIGINT "$RUN_DURATION" "${COMMAND[@]}"; then
        echo "Command exited before the $RUN_DURATION-second window."
    else
        status=$?
        if [ "$status" -eq 124 ]; then
            echo "Time's up. Command was interrupted after $RUN_DURATION seconds."
        elif [ "$status" -ne 130 ]; then
            echo "Command exited with status $status."
        fi
    fi

    echo "Waiting ${COOLDOWN}s for graceful shutdown..."
    sleep "$COOLDOWN"
done
