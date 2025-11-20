#!/usr/bin/env bash

set -euo pipefail

# How long to let each simulation run (in seconds)
RUN_DURATION=${RUN_DURATION:-150} # 2.5 minutes
COOLDOWN=${COOLDOWN:-10}
RUN_COUNT=10
SCENARIO_ID=5

# --- THIS IS THE FIX ---
# Default command is now the LAUNCH script, not the REBUILD script.
COMMAND=("$@")
if [ ${#COMMAND[@]} -eq 0 ]; then
    COMMAND=(./launch_one_sim.sh "$SCENARIO_ID") # <-- CHANGED
fi
# --- END FIX ---


cleanup() {
    echo
    echo "Stopping automated simulation batch run."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Starting automated simulation batch run..."
echo "Command: ${COMMAND[*]}"
echo "Running $RUN_COUNT times, each for $RUN_DURATION seconds."
echo "Press Ctrl+C in this terminal to stop the loop."

# Use a 'for' loop to run 10 times, not 'while true'
for (( i=1; i<=$RUN_COUNT; i++ )); do
    echo "-----------------------------------"
    echo "Starting run $i / $RUN_COUNT at $(date -Iseconds)..."

    if timeout --preserve-status --signal=SIGINT "$RUN_DURATION" "${COMMAND[@]}"; then
        echo "Run $i exited before the $RUN_DURATION-second window."
    else
        status=$?
        if [ "$status" -eq 124 ]; then
            echo "Run $i time's up. Interrupted after $RUN_DURATION seconds."
        elif [ "$status" -ne 130 ]; then
            echo "Run $i exited with status $status."
        fi
    fi

    echo "Waiting ${COOLDOWN}s for graceful shutdown..."
    sleep "$COOLDOWN"
done

echo "-----------------------------------"
echo "Batch run finished."