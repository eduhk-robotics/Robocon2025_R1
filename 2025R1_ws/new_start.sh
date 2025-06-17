#!/bin/bash
# Filename: start.sh

# Go to your workspace root
cd ~/Robocon2025_R1/2025R1_ws

# Set up Python environment
export PYTHONPATH=/home/robotics/Robocon2025_R1/venv/lib/python3.12/site-packages:$PYTHONPATH
export PATH=/home/robotics/Robocon2025_R1/venv/bin:$PATH

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build the workspace
echo "Building workspace in /home/Robocon2025_R1/2025R1_ws..."
colcon build --cmake-args \
  -DPython3_NumPy_INCLUDE_DIRS=$(python -c "import numpy; print(numpy.get_include())") \
  -DPython3_INCLUDE_DIR=$(python -c "from sysconfig import get_paths; print(get_paths()['include'])")
if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

# Source the local workspace
source install/setup.bash

# Check and configure CAN interface (can0)
echo "Checking CAN interface can0..."
if ip link show can0 > /dev/null 2>&1; then
    if ip link show can0 | grep -q "state UP"; then
        echo "CAN interface can0 is already up."
    else
        echo "Configuring CAN interface can0..."
        sudo ip link set can0 down || true
        sudo ip link set can0 up type can bitrate 500000
        if [ $? -eq 0 ]; then
            echo "CAN interface can0 successfully configured."
        else
            echo "Failed to configure CAN interface can0."
            exit 1
        fi
    fi
else
    echo "CAN interface can0 not found. Skipping CAN setup."
fi

# Launch nodes in separate terminals with error handling
launch_node() {
    local package=$1
    local executable=$2
    local name=$3
    echo "Starting $name..."
    xterm -e "ros2 run $package $executable; exec bash" || {
        echo "Failed to start $name. Continuing with other nodes..."
    }
}

launch_node "active_caster" "damiao_node" "damiao_node"
launch_node "active_caster" "vesc_node" "vesc_node"
launch_node "active_caster" "vesc_canbus_speed_control_node" "vesc_canbus_speed_node"
launch_node "navigation" "navigation_node" "navigation_node"
launch_node "navigation" "active_caster_node" "active_caster_node"
launch_node "joystick_driver" "joystick_node" "joystick_node"
launch_node "ps4" "ps4_publisher" "ps4_publisher"

echo "✅ All nodes launched successfully."