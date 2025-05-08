#!/bin/bash
# Filename: start.sh

# Exit immediately on any error
set -e

# Go to your workspace root
cd ~/Robocon2025_R1/2025R1_ws

source ~/Robocon2025_R1/venv/bin/activate

export PYTHONPATH=/home/robotics/Robocon2025_R1/venv/lib/python3.12/site-packages:$PYTHONPATH
export PATH=/home/robotics/Robocon2025_R1/venv/bin:$PATH

# Source ROS 2 environment (adjust 'foxy' if you use galactic, humble, iron, etc.)
source /opt/ros/jazzy/setup.bash

# Build the workspace
echo "Building workspace in /home/Robocon2025_R1/2025R1_ws..."
colcon build --symlink-install

# Source the local workspace
source install/setup.bash

# Launch nodes manuall
# Start data monitor
echo "Starting data monitor..."
gnome-terminal -- bash -c "ros2 run data_monitor data_sender; exec bash"

echo "✅ All nodes launched successfully."
