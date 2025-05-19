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

# Launch nodes manually

# Start damiao_node
echo "Starting damiao_node..."
gnome-terminal -- bash -c "ros2 run damiao damiao_node; exec bash"

# Start navigation_node
echo "Starting active_caster_node..."
gnome-terminal -- bash -c "ros2 run navigation navigation_node; exec bash"

# Start active_caster_node
echo "Starting navigation_node..."
gnome-terminal -- bash -c "ros2 run navigation active_caster_node; exec bash"


# Start ps4_publisher
echo "Starting ps4_publisher..."
gnome-terminal -- bash -c "ros2 run ps4 ps4_publisher; exec bash"

#Start graphic node
#rqt

echo "✅ All nodes launched successfully."
