#!/usr/bin/env python3
"""
VESC CAN control using ASCII-only encoding.
Subscribes to /vesc_control (Float32MultiArray):
  data[0..3] = speeds for CAN IDs 1,2,3,4 (float in [0,383])
Publishes CAN frames via cansend on interface can0.
"""

import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

CAN_CHANNEL = 'can0'
MAX_RPM = 10000
MIN_INPUT = 1.0
MAX_INPUT = 383.0

def map_speed_to_rpm(raw_speed: float) -> int:
    if raw_speed <= 0.0:
        return 0
    norm = (raw_speed - MIN_INPUT) / (MAX_INPUT - MIN_INPUT)
    rpm = int(norm * (MAX_RPM - 1) + 1)
    return max(0, min(rpm, MAX_RPM))

def set_vesc_speed(can_id: int, rpm: int) -> None:
    """
    Send a CAN frame to a VESC at the given CAN ID with the given RPM.
    Uses 29-bit extended frame (0x300 | can_id) and 4-byte big-endian RPM.
    """
    arb_id = f"{0x300 | can_id:08X}"
    data_field = f"{rpm & 0xFFFFFFFF:08X}"
    cmd = f"cansend {CAN_CHANNEL} {arb_id}#{data_field}"
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] failed to send CAN frame: {e}")

class VescCanNode(Node):
    def __init__(self):
        super().__init__('vesc_can_node')
        self.get_logger().info('VescCanNode starting')
        self.create_subscription(
            Float32MultiArray,
            'vesc_control',
            self.control_callback,
            10
        )
        if os.system(f"which cansend > /dev/null") != 0:
            self.get_logger().error("cansend not found; install can-utils")
        self.timer = self.create_timer(0.05, self.heartbeat)
        # last_command: {can_id: rpm}
        self.last_command = {}

    def control_callback(self, msg: Float32MultiArray):
        """
        Callback for /vesc_control messages.
        Expects msg.data length >= 4, data[0..3] ��Ӧ CAN ID 1..4 ���ٶ����롣
        """
        if len(msg.data) < 4:
            self.get_logger().error("vesc_control data must have 4 elements")
            return

        for i in range(4):
            can_id = i + 1
            raw_speed = float(msg.data[i])
            rpm = map_speed_to_rpm(raw_speed)
            self.last_command[can_id] = rpm
            self.get_logger().info(f"Queued ID={can_id} raw_speed={raw_speed:.1f} => RPM={rpm}")

    def heartbeat(self):
        """
        Periodically send last commanded RPM to each VESC.
        """
        for can_id, rpm in self.last_command.items():
            set_vesc_speed(can_id, rpm)

    def destroy_node(self):
        """
        Stop all VESCs on shutdown by sending RPM=0.
        """
        for can_id in list(self.last_command.keys()):
            set_vesc_speed(can_id, 0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VescCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
