#!/bin/bash

# This script runs your simulation in an infinite loop
# with a 3-minute (180 second) timer for each run.
# Press Ctrl+C in this terminal to stop the loop.

# How long to let each simulation run (in seconds)
RUN_DURATION=180 # 3 minutes

echo "Starting automated simulation batch run..."
echo "Each run will last $RUN_DURATION seconds."
echo "Press Ctrl+C in THIS terminal (not the launch window) to stop the loop."

# Infinite loop
while true
do
    echo "-----------------------------------"
    echo "Starting new simulation run..."
    
    # Start your launch script in the background
    ./rebuild_and_launch.sh &
    
    # Get the Process ID (PID) of the launch script
    LAUNCH_PID=$!
    
    echo "Simulation running with PID $LAUNCH_PID. Waiting $RUN_DURATION seconds..."
    
    # Wait for 3 minutes
    sleep $RUN_DURATION
    
    echo "Time's up. Shutting down simulation (PID $LAUNCH_PID)..."
    
    # Send a graceful shutdown signal (SIGINT), just like pressing Ctrl+C
    kill -SIGINT $LAUNCH_PID
    
    # Wait 10 seconds for everything to save (rosbags, parquet)
    echo "Waiting 10 seconds for graceful shutdown..."
    sleep 10
    echo "-----------------------------------"
done