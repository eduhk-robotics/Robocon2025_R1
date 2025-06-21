#!/usr/bin/env python3
"""
ActiveCasterNode (No alignment offset)
Swerve-drive kinematics with simultaneous translation and rotation.

Publishes:
    - damiao_control (Float32MultiArray) - steering angle commands
    - vesc_control   (Float32MultiArray) - per-wheel RPM commands
    - ai_order       (Float32MultiArray) - AI order flag (r1 button)
Subscribes:
    - driving        (Float32MultiArray) - [heading_deg, speed_raw, rotation_raw]
    - joystick_input (Joystick)           - joystick 按键和摇杆输入
    - /basket_detector/status (String)    - 篮筐位置状态：TURN_LEFT, GO_STRAIGHT, TURN_RIGHT
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from joystick_msgs.msg import Joystick

# Encoder / gear constants
ENCODER_RATIO = 19.20321         # encoder counts per motor revolution
GEAR_RATIO    = 61.0 / 35.0      # wheel_rev = motor_rev / GEAR_RATIO
TWO_PI        = 2.0 * math.pi
# encoder_rad per wheel revolution
PERIOD_RAD    = TWO_PI * ENCODER_RATIO * GEAR_RATIO

# Geometry and speed limits
DIAMOND_RADIUS   = 0.267552      # m, distance from center in diamond layout
WHEEL_RADIUS     = 0.05          # m, wheel radius

# Alignment offset removed
WHEEL_ALIGNMENT_OFFSET = 0.0

# Hardware servo ID mapping
SERVO_ID_MAP = [2, 3, 4, 1]

# Diamond layout positions
WHEEL_POS = [
    ( DIAMOND_RADIUS,   0.0),
    ( 0.0,             -DIAMOND_RADIUS),
    (-DIAMOND_RADIUS,   0.0),
    ( 0.0,              DIAMOND_RADIUS),
]

MAX_LINEAR_MPS  = 2.0
MAX_ANGULAR_RPS = math.pi
MAX_RPM         = 9000

# Sentinel: if speed_raw equals this, we trigger "stop" behavior
SENTINEL_SPEED = 114514520.0

class ActiveCasterNode(Node):
    def __init__(self):
        super().__init__('active_caster_node')
        self.get_logger().info('ActiveCasterNode starting')

        # 记录篮筐状态
        self.basket_status = None  # "TURN_LEFT", "GO_STRAIGHT", "TURN_RIGHT"

        # Steering & RPM ramp state
        self.last_pos_rad     = [0.0] * 4
        self.current_rpm      = [0.0] * 4
        self.target_rpm       = [0.0] * 4
        self.ramp_initial_rpm = [0.0] * 4
        self.ramp_start_time  = None
        self.ramp_duration    = 0.5  # seconds (加速/减速时间延长为1s)

        # Publishers & subscribers
        self.pos_pub = self.create_publisher(Float32MultiArray, 'damiao_control', 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, 'vesc_control', 10)
        self.ai_pub  = self.create_publisher(Float32MultiArray, 'ai_order', 10)

        self.create_subscription(Float32MultiArray, 'driving', self.driving_callback, 10)
        self.create_subscription(Joystick, 'joystick_input', self.joystick_callback, 10)
        self.create_subscription(String, '/basket_detector/status', self.basket_callback, 10)

        # Timer for RPM interpolation (50 Hz)
        self.create_timer(0.02, self.timer_callback)

    @staticmethod
    def angle_to_encoder_rad(angle_deg):
        wheel_rev   = angle_deg / 360.0
        motor_rev   = wheel_rev * GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        return encoder_rev * TWO_PI

    def publish_steering(self, servo_id, target_rad):
        msg = Float32MultiArray()
        # [servo_id, mode, speed, angle]
        msg.data = [float(servo_id), 2.0, 20.0, target_rad]
        self.pos_pub.publish(msg)

    def basket_callback(self, msg: String):
        if msg.data in ('TURN_LEFT', 'GO_STRAIGHT', 'TURN_RIGHT'):
            self.basket_status = msg.data

    def driving_callback(self, msg: Float32MultiArray):
        heading_deg  = float(msg.data[0])
        speed_raw    = float(msg.data[1])
        rotation_raw = float(msg.data[2])

        # —— 哨兵检测 ——
        if speed_raw == SENTINEL_SPEED:
            for idx, last_rad in enumerate(self.last_pos_rad):
                servo_id = SERVO_ID_MAP[idx]
                self.publish_steering(servo_id, last_rad)
            stop_msg = Float32MultiArray(); stop_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.rpm_pub.publish(stop_msg)

            self.current_rpm      = [0.0] * 4
            self.target_rpm       = [0.0] * 4
            self.ramp_initial_rpm = [0.0] * 4
            self.ramp_start_time  = None
            return

        # 常规驱动计算
        # Translation vector
        v_norm = max(min(speed_raw / 8192.0, 1.0), 0.0)
        v_mag  = v_norm * MAX_LINEAR_MPS
        h_rad  = math.radians(heading_deg)
        Vt_x   = v_mag * math.cos(h_rad)
        Vt_y   = v_mag * math.sin(h_rad)

        # Yaw rate
        w_norm = max(min(rotation_raw / 8192.0, 1.0), -1.0)
        omega  = w_norm * MAX_ANGULAR_RPS

        new_rpm_targets = []
        for i, (rx, ry) in enumerate(WHEEL_POS, start=1):
            Vr_x = -omega * ry
            Vr_y =  omega * rx
            Vi_x = Vt_x + Vr_x
            Vi_y = Vt_y + Vr_y

            steer = math.atan2(Vi_y, Vi_x) + WHEEL_ALIGNMENT_OFFSET
            steer = (steer + math.pi) % (2*math.pi) - math.pi

            speed = math.hypot(Vi_x, Vi_y)
            rpm   = (speed / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0
            new_rpm_targets.append(rpm)

            servo_id = SERVO_ID_MAP[i-1]
            tgt_rad  = self.angle_to_encoder_rad(math.degrees(steer))

            # shortest path
            delta = tgt_rad - self.last_pos_rad[i-1]
            if delta > PERIOD_RAD/2.0:
                tgt_rad -= PERIOD_RAD
            elif delta < -PERIOD_RAD/2.0:
                tgt_rad += PERIOD_RAD

            self.publish_steering(servo_id, tgt_rad)
            self.last_pos_rad[i-1] = tgt_rad

        # limit rpm
        max_r = max(new_rpm_targets)
        if max_r > MAX_RPM:
            scale = MAX_RPM / max_r
            new_rpm_targets = [r * scale for r in new_rpm_targets]

        # 对 rpm 目标做循环右移
        new_rpm_targets = [
            new_rpm_targets[1],
            new_rpm_targets[2],
            new_rpm_targets[3],
            new_rpm_targets[0],
        ]

        # ramp into new targets if changed
        if any(abs(n - t) > 1e-1 for n, t in zip(new_rpm_targets, self.target_rpm)):
            self.ramp_initial_rpm = list(self.current_rpm)
            self.target_rpm       = list(new_rpm_targets)
            self.ramp_start_time  = self.get_clock().now()

    def joystick_callback(self, msg: Joystick):
        # 发布 R1 标志
        val = 1.0 if msg.r1 else 0.0
        out = Float32MultiArray(); out.data = [val]
        self.ai_pub.publish(out)

        if msg.r1:
            # 如果正在调整，第二次按下 R1 则停止过程
            if self.ramp_start_time is not None:
                # 保持当前舵机角度
                for idx, last_rad in enumerate(self.last_pos_rad):
                    servo_id = SERVO_ID_MAP[idx]
                    self.publish_steering(servo_id, last_rad)
                # 发布停止 RPM
                stop_msg = Float32MultiArray(); stop_msg.data = [0.0, 0.0, 0.0, 0.0]
                self.rpm_pub.publish(stop_msg)
                # 重置 ramp 状态
                self.current_rpm      = [0.0] * 4
                self.target_rpm       = [0.0] * 4
                self.ramp_initial_rpm = [0.0] * 4
                self.ramp_start_time  = None
                return
            # 首次按下 R1，正常注入自转指令
            if self.basket_status is not None:
                auto = Float32MultiArray()
                if self.basket_status == 'TURN_LEFT':
                    rotation = -8192.0
                elif self.basket_status == 'TURN_RIGHT':
                    rotation = 8192.0
                else:
                    rotation = 0.0
                auto.data = [0.0, 0.0, rotation]
                self.driving_callback(auto)

    def timer_callback(self):
        if self.ramp_start_time is None:
            msg = Float32MultiArray(); msg.data = self.current_rpm
            self.rpm_pub.publish(msg)
            return

        elapsed = (self.get_clock().now() - self.ramp_start_time).nanoseconds * 1e-9
        t       = min(max(elapsed / self.ramp_duration, 0.0), 1.0)
        # quintic smoothstep for extra smooth accel/decel
        s       = t**3 * (t*(6*t - 15) + 10)
        self.current_rpm = [
            r0 + s*(r1 - r0)
            for r0, r1 in zip(self.ramp_initial_rpm, self.target_rpm)
        ]

        msg = Float32MultiArray(); msg.data = self.current_rpm
        self.rpm_pub.publish(msg)

        if t >= 1.0:
            self.ramp_start_time = None


def main(args=None):
    rclpy.init(args=args)
    node = ActiveCasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
