#!/bin/bash
# Filename: start.sh

# Exit immediately on any error
set -e

# Go to your workspace root
cd ~/Robocon2025_R1/2025R1_ws

#export PYTHONPATH=/home/robotics/Robocon2025_R1/venv/lib/python3.12/site-packages:$PYTHONPATH
#export PATH=/home/robotics/Robocon2025_R1/venv/bin:$PATH

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build the workspace
#echo "Building workspace in /home/Robocon2025_R1/2025R1_ws..."
#colcon build --symlink-install

# Source the local workspace
source install/setup.bash

# Check and configure CAN interface (can0) only if it's not already up
echo "Checking CAN interface can0..."
if ip link show can0 | grep -q "state UP"; then
    echo "CAN interface can0 is already up."
else
    echo "Configuring CAN interface can0..."
    sudo ip link set can0 down || true  # Bring it down if it’s in an unknown state
    sudo ip link set can0 up type can bitrate 500000
    if [ $? -eq 0 ]; then
        echo "CAN interface can0 successfully configured."
    else
        echo "Failed to configure CAN interface can0."
        exit 1
    fi
fi

# Launch nodes manually

# Start damiao_node
echo "Starting damiao_node..."
gnome-terminal -- bash -c "ros2 run active_caster damiao_node; exec bash"

# Start vesc_node
#echo "Starting vesc_node..."
#gnome-terminal -- bash -c "ros2 run active_caster vesc_node; exec bash"

# Start vesc_canbus_speed_node
echo "Starting vesc_canbus_speed_node..."
gnome-terminal -- bash -c "ros2 run active_caster vesc_canbus_speed_control_node; exec bash"

gnome-terminal -- bash -c "ros2 run auto_move_node; exec bash"

# Start navigation_node
echo "Starting navigation_node..."
gnome-terminal -- bash -c "ros2 run navigation navigation_node; exec bash"

# Start active_caster_node
echo "Starting active_caster_node..."
gnome-terminal -- bash -c "ros2 run navigation active_caster_node; exec bash"

# Start joystick_node
echo "Starting joystick_node..."
gnome-terminal -- bash -c "ros2 run joystick_driver joystick_node; exec bash"

# Start shooter_node
echo "Starting shooter_node..."
gnome-terminal -- bash -c "ros2 run shooter shooter_control_node; exec bash"
gnome-terminal -- bash -c "ros2 run shooter shooter_vesc_node; exec bash"
#gnome-terminal -- bash -c "ros2 run shooter shooter_damiao_node; exec bash"


# Start bounce
gnome-terminal -- bash -c "ros2 run bounce bounce_control_node; exec bash"
gnome-terminal -- bash -c "ros2 run bounce bounce_relays_node; exec bash"

echo "✅ All nodes launched successfully."
