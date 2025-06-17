#!/bin/bash

source ~/Robocon2025_R1/2025R1_ws/install/setup.bash

echo "检查 can0..."
if ip link show can0 | grep -q "state UP"; then
    echo "can0 已启动"
else
    echo "配置 can0..."
    sudo ip link set can0 down || true
    sudo ip link set can0 up type can bitrate 500000
    [ $? -eq 0 ] && echo "can0 配置成功" || { echo "can0 配置失败"; exit 1; }
fi