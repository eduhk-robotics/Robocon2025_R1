import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import serial
import struct
import os
import time

# To discover your USB-CDC device ID, run:
#     ls /dev/serial/by-id/
# and note the entry that appears when your device is plugged in.
# Example device ID: usb-HDSC_CDC_Device_00000000050C-if00
DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"

def find_device_port(device_id):
    """Scan /dev/serial/by-id for our device and return its real tty path, or None."""
    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        pass
    return None

# Wait until the device shows up
print(f"Looking for USB serial device with ID '{DEVICE_ID}'…")
port = None
while port is None:
    port = find_device_port(DEVICE_ID)
    if port is None:
        print("  Device not found, retrying in 1 second…")
        time.sleep(1)
    else:
        print(f"Found device on {port}")

# Open the device once found
ser = serial.Serial(port, 921600, timeout=1)
if ser.isOpen():
    print("Successfully opened CDC device")

# Constants and flags for motor control
sendControl_flag = 0
sendCommed_flag = 0
data1 = []
CMD_MOTOR_MODE = 0x01
CMD_RESET_MODE = 0x02
CMD_ZERO_POSITION = 0x03
CMD_START_MODE = 0x04
send_counter = 0
send_datas = np.zeros((54, 1), np.uint8)
control_datas = np.zeros((58, 1), np.uint8)

P_MIN = -95.5
P_MAX = 95.5
V_MIN = -45.0
V_MAX = 45.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -13.0
T_MAX = 13.0

A_MIN = 0.0
A_MAX = 20.0  # Rated current 5A, stall current 20A. Torque constant = 0.28, Torque = constant * A
motor_feedback = np.zeros((12, 4))

s_flag = 0

# Buffers for CAN communication
CAN1_buf = np.zeros((9, 1), np.uint8)
CAN1_buf1 = np.zeros((13, 1), np.uint8)
CAN1_buf2 = np.zeros((13, 1), np.uint8)
CAN1_buf3 = np.zeros((13, 1), np.uint8)

CAN2_buf = np.zeros((9, 1), np.uint8)
CAN2_buf1 = np.zeros((13, 1), np.uint8)
CAN2_buf2 = np.zeros((13, 1), np.uint8)
CAN2_buf3 = np.zeros((13, 1), np.uint8)

# Initialize CAN buffers
CAN1_buf1[0] = CAN1_buf2[0] = CAN1_buf3[0] = 8
CAN1_buf1[1] = CAN1_buf2[1] = CAN1_buf3[1] = 0
CAN1_buf1[2] = CAN1_buf2[2] = CAN1_buf3[2] = 0
CAN1_buf1[3] = CAN1_buf2[3] = CAN1_buf3[3] = 0

CAN2_buf1[0] = CAN2_buf2[0] = CAN2_buf3[0] = 8
CAN2_buf1[1] = CAN2_buf2[1] = CAN2_buf3[1] = 0
CAN2_buf1[2] = CAN2_buf2[2] = CAN2_buf3[2] = 0
CAN2_buf1[3] = CAN2_buf2[3] = CAN2_buf3[3] = 0

# Function to calculate Modbus CRC16
def GetModbusCRC16_Cal(data, length):
    temp = 0
    wcrc = 0xffff
    for i in range(length):
        temp = data[i] & 0x00ff
        wcrc ^= temp
        for j in range(8):
            if wcrc & 0x0001:
                wcrc >>= 1
                wcrc ^= 0xA001
            else:
                wcrc >>= 1
    return (wcrc << 8) | (wcrc >> 8)

# Functions to convert between float and unsigned integer
def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return np.uint16((x - offset) * ((1 << bits) - 1) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return np.float(x_int * span / ((1 << bits) - 1) + offset)

def LIMIT_MIN_MAX(x, min_val, max_val):
    if x <= min_val:
        x = min_val
    elif x > max_val:
        x = max_val

# Function to process received data
receive_data = np.zeros((6, 8), np.uint8)
def receive_process(data):
    global receive_data
    if data[0] == 100 and data[1] == 99:
        receive_data[0] = data[2:10]
        receive_data[1] = data[10:18]
        receive_data[2] = data[18:26]
        receive_data[3] = data[26:34]
        receive_data[4] = data[34:42]
        receive_data[5] = data[42:50]
        for i in range(6):
            temp_value = (np.uint16(receive_data[i][1]) << 8) | np.uint16(receive_data[i][2])
            CurPosition = uint_to_float(temp_value, P_MIN, P_MAX, 16)
            temp_value1 = (np.uint16(receive_data[i][3]) << 4) | ((receive_data[i][4] & 0xf0) >> 4)
            CurVelocity = uint_to_float(temp_value1, V_MIN, V_MAX, 12)
            temp_value2 = ((np.uint16(receive_data[i][4]) & 0x000f) << 8) | np.uint16(receive_data[i][5])
            CurTorque = 0.28 * uint_to_float(temp_value2, A_MIN, A_MAX, 12)
            motor_feedback[receive_data[i][0] - 1, 0] = receive_data[i][0]
            motor_feedback[receive_data[i][0] - 1, 1] = CurPosition
            motor_feedback[receive_data[i][0] - 1, 2] = CurVelocity
            motor_feedback[receive_data[i][0] - 1, 3] = CurTorque

# CAN communication functions for sending control parameters
buf = np.zeros((9, 1), np.uint8)
buf1 = np.zeros((11, 1), np.uint8)
def CanComm_SendControlPara(f_p, f_v, f_kp, f_kd, f_t, GM8115_ID):
    global sendControl_flag, pre_time, s_flag, CAN1_buf1
    send_data = np.array([0x55, 0xAA, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x88], np.uint8)

    LIMIT_MIN_MAX(f_p, P_MIN, P_MAX)
    LIMIT_MIN_MAX(f_v, V_MIN, V_MAX)
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX)
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX)
    LIMIT_MIN_MAX(f_t, T_MIN, T_MAX)
    p = float_to_uint(f_p, P_MIN, P_MAX, 16)
    v = float_to_uint(f_v, V_MIN, V_MAX, 12)
    kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12)
    kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12)
    t = float_to_uint(f_t, T_MIN, T_MAX, 12)
    buf[0] = GM8115_ID
    buf[1] = p >> 8
    buf[2] = p & 0xFF
    buf[3] = v >> 4
    buf[4] = ((v & 0xF) << 4) | (kp >> 8)
    buf[5] = kp & 0xFF
    buf[6] = kd >> 4
    buf[7] = ((kd & 0xF) << 4) | (t >> 8)
    buf[8] = t & 0xFF
    send_data[13] = GM8115_ID
    send_data[21:29] = buf[1:9, 0]
    print(send_data)
    ser.write(bytes(send_data.T))

# Convert degrees to radians
def _deg2rad(value_deg: float) -> float:
    """Convert degrees → radians."""
    return np.deg2rad(value_deg)

# Function to send speed control command to CAN bus
def CanComm_SendSpeed(v_des, GM8115_ID):
    """
    Send a single speed-mode CAN frame.

    Parameters
    ----------
    v_des : float        # target speed [rad/s]
    GM8115_ID : int      # Motor CAN-ID (1 to 2047)
    """
    # Build the CAN payload
    v_des = _deg2rad(v_des)
    can_id = 0x200 + GM8115_ID
    payload4 = struct.pack("<f", float(v_des))

    # Wrap in the 30-byte USB frame
    send_data = np.zeros((30,), np.uint8)
    send_data[0:13] = [0x55, 0xAA, 0x1E, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00]
    send_data[13] = can_id & 0xFF
    send_data[14] = (can_id >> 8) & 0xFF
    send_data[18] = 0x04
    send_data[21:25] = list(payload4)
    send_data[29] = 0x88

    # Transmit over USB-CDC
    ser.write(bytes(send_data))

# Function to send position-velocity control command to CAN bus
def CanComm_SendPosition(p_des, v_des, GM8115_ID):
    p_des = _deg2rad(p_des)
    v_des = _deg2rad(v_des)
    """
    Position-Velocity command (position-velocity mode).

    Parameters
    ----------
    p_des : float   # Target position [rad]
    v_des : float   # Max speed limit [rad/s] (0 → no limit)
    GM8115_ID : int # Motor CAN-ID (1 to 2047)
    """
    # Build the CAN frame
    can_id = 0x100 + GM8115_ID
    payload8 = struct.pack("<ff", float(p_des), float(v_des))

    # Wrap in the 30-byte USB message
    msg = np.zeros(30, dtype=np.uint8)
    msg[0:13] = [0x55, 0xAA, 0x1E, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00]
    msg[29] = 0x88
    msg[13] = can_id & 0xFF
    msg[14] = (can_id >> 8) & 0xFF
    msg[18] = 0x08
    msg[21:29] = list(payload8)

    # Transmit over USB-CDC
    ser.write(bytes(msg))
    ser.flush()

# Function to send control command (reset, motor mode, etc.)
def CanComm_ControlCmd(cmd, GM8115_ID):
    global CAN1_buf1, CAN1_buf2
    global sendCommed_flag
    send_data = np.array([0x55, 0xAA, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x88], np.uint8)
    buf = np.array([GM8115_ID, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00], np.uint8)
    if cmd == CMD_MOTOR_MODE:
        buf[8] = 0xFC
    elif cmd == CMD_RESET_MODE:
        buf[8] = 0xFD
    elif cmd == CMD_ZERO_POSITION:
        buf[8] = 0xFE
    else:
        print("Waiting for motor startup command")
    send_data[13] = GM8115_ID
    send_data[21:29] = buf[1:9]
    print(send_data)
    ser.write(bytes(send_data.T))

# Functions for motor control (start/stop, position/speed)
def ZeroPosition(GM8115_ID):
    CanComm_ControlCmd(CMD_MOTOR_MODE, GM8115_ID)
    CanComm_SendControlPara(0, 0, 0, 0, 0, GM8115_ID)

def MotorControl_Start(GM8115_ID):
    ZeroPosition(GM8115_ID)
    CanComm_ControlCmd(CMD_ZERO_POSITION, GM8115_ID)

def MotorControl_Stop(GM8115_ID):
    CanComm_ControlCmd(CMD_RESET_MODE, GM8115_ID)

# Example function to control motor start/stop
def MotorControl_startSend(GM8115_ID):
    motor_commed = np.zeros((10, 1), np.uint8)
    motor_commed[0] = 88
    motor_commed[1] = 87
    for i in range(2, 8):
        motor_commed[i] = GM8115_ID[i - 2]
    S_CRC_result = GetModbusCRC16_Cal(motor_commed, 8)
    motor_commed[8] = (S_CRC_result & 0xff00) >> 8
    motor_commed[9] = (S_CRC_result & 0x00ff)

def MotorControl_stopSend(GM8115_ID):
    motor_commedStop = np.zeros((10, 1), np.uint8)
    motor_commedStop[0] = 78
    motor_commedStop[1] = 77
    for i in range(2, 8):
        motor_commedStop[i] = GM8115_ID[i - 2]
    S_CRC_result = GetModbusCRC16_Cal(motor_commedStop, 8)
    motor_commedStop[8] = (S_CRC_result & 0xff00) >> 8
    motor_commedStop[9] = (S_CRC_result & 0x00ff)

# Stop all motors initially
for i in range(1, 9):
    MotorControl_Stop(i)

# Motor control functions for use
def stop_motor(motor_id):
    MotorControl_Stop(motor_id)

def pos_move(motor_id, pos, speed):
    MotorControl_Start(motor_id)  # Start Motor
    CanComm_SendPosition(pos, speed, motor_id)

def speed_move(motor_id, speed):
    MotorControl_Start(motor_id)  # Start Motor
    CanComm_SendSpeed(speed, motor_id)


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('damiao_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'damiao',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Motor controller node started, listening on 'damiao' topic.")

    def listener_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn("Received message with insufficient data.")
            return

        motor_id = int(msg.data[0])
        mode = int(msg.data[1])
        speed = float(msg.data[2])
        position = float(msg.data[3])

        self.get_logger().info(f"Received command - ID: {motor_id}, Mode: {mode}, Speed: {speed}, Pos: {position}")

        if mode == 0:
            stop_motor(motor_id)
        elif mode == 1:
            speed_move(motor_id, speed)
        elif mode == 2:
            pos_move(motor_id, position, speed)
        else:
            self.get_logger().warn(f"Unknown mode received: {mode}")


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    rclpy.spin(motor_controller_node)

    motor_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
