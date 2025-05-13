#!/usr/bin/env python3
import math
import time
import serial
from serial.tools import list_ports
import pyvesc
from pyvesc import GetValues, SetRPM

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Constants for steering servos
ENCODER_RATIO = 19.20321
GEAR_RATIO = 35.0 / 61.0
TWO_PI = 2.0 * math.pi
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # ~210 rad

# Constants for drive motors
MAX_RPM = 6000                   # max wheel RPM
MAX_INPUT = 8000.0               # max input command value
HEARTBEAT_DT = 0.05              # s between VESC heartbeats
DESIRED_IDS = {1, 2, 3, 4}       # CAN IDs for each wheel motor

# Geometry (units consistent with control inputs)
a = 19.0  # front axle to CG distance
c = 19.0  # CG to rear axle distance
b = 19.0  # half track width
L = a + c

# Wheel positions relative to CG
# motor_id: (x_i, y_i)
WHEEL_POS = {
    1: ( a,  b),  # front-left
    2: ( a, -b),  # front-right
    3: (-c,  b),  # rear-left
    4: (-c, -b),  # rear-right
}

class OmniWheelController(Node):
    def __init__(self):
        super().__init__('omni_wheel_controller')

        # Subscribe to driving commands: [steer_deg, speed, yaw]
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # Publisher for steering servos
        self.pos_pub = self.create_publisher(
            Float32MultiArray,
            'damiao_control',
            10
        )

        # Track last steering positions to wrap angles
        self.last_pos_rad = {i: 0.0 for i in WHEEL_POS}

        # Discover and open serial ports for VESCs
        self.port_map = self.find_vescs()
        if not self.port_map:
            self.get_logger().error(f'No VESCs with CAN IDs {DESIRED_IDS} found')
        self.serial_map = {}
        for dev in self.port_map:
            ser = serial.Serial(dev, baudrate=115200, timeout=0.1)
            time.sleep(0.1)
            self.serial_map[dev] = ser

        # Desired RPM for each drive motor
        self.desired_rpms = {i: 0 for i in WHEEL_POS}

        # Heartbeat timer to continuously send RPM commands
        self.create_timer(HEARTBEAT_DT, self.heartbeat)

    def scan_ids_on_port(self, dev):
        found = []
        try:
            with serial.Serial(dev, 115200, timeout=0.1) as ser:
                time.sleep(0.1)
                for can_id in DESIRED_IDS:
                    setattr(GetValues, 'can_id', can_id)
                    ser.write(pyvesc.encode_request(GetValues))
                    time.sleep(HEARTBEAT_DT)
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        msg, _ = pyvesc.decode(data)
                        if msg:
                            found.append(can_id)
        except Exception as e:
            self.get_logger().warn(f'[scan] error on {dev}: {e}')
        return found

    def find_vescs(self):
        ports = [p.device for p in list_ports.comports() if 'ACM' in p.device or 'USB' in p.device]
        res = {}
        for dev in ports:
            ids = self.scan_ids_on_port(dev)
            if ids:
                res[dev] = ids
                self.get_logger().info(f'Found VESCs {ids} on {dev}')
        return res

    def driving_callback(self, msg: Float32MultiArray):
        # Unpack inputs
        steer_deg = msg.data[0]
        speed_cmd = msg.data[1] / MAX_INPUT      # normalize to [-1..1]
        yaw_cmd   = msg.data[2] / MAX_INPUT      # normalize to [-1..1]

        # --- 1) Steering angle calculation ---
        phi = math.radians(steer_deg)
        t = math.tan(phi)

        # Compute per-wheel steering angles (rad)
        delta = {
            1: math.atan( t/(2 - t) ),
            2: math.atan( t/(2 + t) ),
            3: -math.atan( t/(2 - t) ),
            4: -math.atan( t/(2 + t) ),
        }

        # Publish servo commands
        for i, angle in delta.items():
            # convert to encoder radians
            wheel_rev   = math.degrees(angle) / 360.0
            motor_rev   = wheel_rev / GEAR_RATIO
            encoder_rev = motor_rev * ENCODER_RATIO
            target_rad  = encoder_rev * TWO_PI

            # wrap to nearest
            diff = target_rad - self.last_pos_rad[i]
            if diff >  PERIOD_RAD/2:
                target_rad -= PERIOD_RAD
            elif diff < -PERIOD_RAD/2:
                target_rad += PERIOD_RAD

            msg_out = Float32MultiArray()
            msg_out.data = [float(i), 2.0, 40.0, target_rad]
            self.pos_pub.publish(msg_out)
            self.last_pos_rad[i] = target_rad

        # --- 2) Drive RPM calculation ---
        # Treat speed_cmd as longitudinal v, yaw_cmd as angular rate omega
        v = speed_cmd
        omega = yaw_cmd

        for i, (x_i, y_i) in WHEEL_POS.items():
            # local velocity vector
            vx = v - omega * y_i
            vy =     omega * x_i
            lin_norm = math.hypot(vx, vy)

            # map to rpm
            rpm = int(lin_norm * MAX_RPM)
            # preserve sign along wheel direction
            # projection onto wheel orientation
            orient = delta[i]
            direction = math.cos(orient)*vx + math.sin(orient)*vy
            if direction < 0:
                rpm = -rpm

            self.desired_rpms[i] = rpm

    def heartbeat(self):
        # send SetRPM to each VESC
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                rpm = self.desired_rpms.get(can_id, 0)
                cmd = SetRPM(rpm)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))

    def destroy_node(self):
        # stop motors on shutdown
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetRPM(0)
                cmd.can_id = can_id
                ser.write(pyvesc.encode(cmd))
        for ser in self.serial_map.values():
            ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
