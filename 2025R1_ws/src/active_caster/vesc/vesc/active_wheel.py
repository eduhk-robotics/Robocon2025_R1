import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from serial.tools import list_ports
import pyvesc
from pyvesc.VESC.messages import GetValues, SetRPM, SetCurrentBrake
import time

# Constants
BAUDRATE = 115200
TIMEOUT = 0.1
HEARTBEAT_DT = 0.05
MAX_RPM = 20000
DESIRED_IDS = {1, 3, 4}

class ActiveWheel(Node):
    def __init__(self):
        super().__init__('active_wheel')
        self.get_logger().info('ActiveWheel starting')
        self.desired_rpm = 0
        self.last_input = time.time()

        # Subscribe to vesc_control (RPM 0-10000)
        self.create_subscription(
            Float32MultiArray,
            'vesc_control',
            self.callback,
            10
        )

        # Find VESCs
        self.port_map = self.find_vescs()
        if not self.port_map:
            self.get_logger().error(f'No VESCs with CAN IDs {DESIRED_IDS} found')
            return

        # Open serial ports
        self.serial_map = {}
        for dev in self.port_map:
            ser = serial.Serial(dev, BAUDRATE, timeout=TIMEOUT)
            time.sleep(0.1)
            self.serial_map[dev] = ser

        # Start heartbeat
        self.create_timer(HEARTBEAT_DT, self.heartbeat)

    def scan_ids_on_port(self, dev):
        found = []
        try:
            with serial.Serial(dev, BAUDRATE, timeout=TIMEOUT) as ser:
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
        ports = [
            p.device for p in list_ports.comports()
            if 'ACM' in p.device or 'USB' in p.device
        ]
        res = {}
        for dev in ports:
            ids = self.scan_ids_on_port(dev)
            if ids:
                res[dev] = ids
                self.get_logger().info(f'Found VESCs {ids} on {dev}')
        return res

    def callback(self, msg: Float32MultiArray):
        if msg.data:
            try:
                self.desired_rpm = int(msg.data[0])
                self.last_input = time.time()
                self.get_logger().info(f'RPM: {self.desired_rpm}')
            except (ValueError, IndexError):
                self.get_logger().warn('Invalid vesc_control data')
                self.desired_rpm = 0

    def heartbeat(self):
        # Timeout: stop motors after 5 seconds
        if time.time() - self.last_input > 5:
            self.desired_rpm = 0
            self.get_logger().info('Timeout, stopping motors')

        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetCurrentBrake(10000) if self.desired_rpm == 0 else SetRPM(self.desired_rpm)
                cmd.can_id = can_id
                try:
                    ser.write(pyvesc.encode(cmd))
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial error on {dev}: {e}')

    def destroy_node(self):
        # Stop motors
        for dev, ids in self.port_map.items():
            ser = self.serial_map[dev]
            for can_id in ids:
                cmd = SetCurrentBrake(10000)
                cmd.can_id = can_id
                try:
                    ser.write(pyvesc.encode(cmd))
                except serial.SerialException:
                    self.get_logger().warn(f'Failed to send brake on {dev}')
        # Close serial ports
        for ser in self.serial_map.values():
            ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ActiveWheel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()