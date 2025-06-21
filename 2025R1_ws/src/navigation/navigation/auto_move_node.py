#!/usr/bin/env python3
"""
Single-file ROS2 node that implements ActiveCasterNode and aligns to a detected basket.
Subscribes to:
  - /basket_detector/status (std_msgs/String) with values:
      TURN_LEFT, TURN_RIGHT, GO_STRAIGHT
  - ai_order (std_msgs/Float32MultiArray), where data[0] transitions from 0→1 to trigger alignment.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

# Encoder / gear constants
ENCODER_RATIO = 19.20321         # encoder counts per motor revolution
GEAR_RATIO    = 61.0 / 35.0      # wheel_rev = motor_rev / GEAR_RATIO
TWO_PI        = 2.0 * math.pi
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

# Sentinel: if speed_raw equals this, we trigger “stop” behavior
SENTINEL_SPEED = 114514520.0

class AutoMoveNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.get_logger().info(f'{name} starting')

        # Steering & RPM state
        self.last_pos_rad     = [0.0] * 4
        self.current_rpm      = [0.0] * 4
        self.target_rpm       = [0.0] * 4
        self.ramp_initial_rpm = [0.0] * 4
        self.ramp_start_time  = None
        self.ramp_duration    = 0.5  # seconds

        # Publishers & subscriber
        self.pos_pub = self.create_publisher(Float32MultiArray, 'damiao_control', 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, 'vesc_control', 10)
        self.create_subscription(Float32MultiArray, 'driving', self.driving_callback, 10)

        # Timer for RPM interpolation
        self.create_timer(0.02, self.timer_callback)  # 50 Hz

    @staticmethod
    def angle_to_encoder_rad(angle_deg: float) -> float:
        wheel_rev   = angle_deg / 360.0
        motor_rev   = wheel_rev * GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        return encoder_rev * TWO_PI

    def publish_steering(self, servo_id: int, target_rad: float):
        msg = Float32MultiArray()
        # [servo_id, mode=2 (position), speed=20, angle_rad]
        msg.data = [float(servo_id), 2.0, 20.0, target_rad]
        self.pos_pub.publish(msg)

    def driving_callback(self, msg: Float32MultiArray):
        heading_deg  = float(msg.data[0])
        speed_raw    = float(msg.data[1])
        rotation_raw = float(msg.data[2])

        # —— Sentinel: immediate stop ——
        if speed_raw == SENTINEL_SPEED:
            # 1) Hold last steering
            for idx, last_rad in enumerate(self.last_pos_rad):
                self.publish_steering(SERVO_ID_MAP[idx], last_rad)
            # 2) Zero RPM
            self.rpm_pub.publish(Float32MultiArray(data=[0.0]*4))
            # Reset ramp state
            self.current_rpm      = [0.0]*4
            self.target_rpm       = [0.0]*4
            self.ramp_initial_rpm = [0.0]*4
            self.ramp_start_time  = None
            return

        # Translation vector
        v_norm = max(min(speed_raw / 8192.0, 1.0), 0.0)
        v_mag  = v_norm * MAX_LINEAR_MPS
        h_rad  = math.radians(heading_deg)
        Vt_x   = v_mag * math.cos(h_rad)
        Vt_y   = v_mag * math.sin(h_rad)

        # Yaw rate
        w_norm = max(min(rotation_raw / 8192.0, 1.0), -1.0)
        omega  = w_norm * MAX_ANGULAR_RPS

        # Compute new RPM targets & steering
        new_rpm = []
        for i, (rx, ry) in enumerate(WHEEL_POS):
            Vr_x = -omega * ry
            Vr_y =  omega * rx
            Vi_x = Vt_x + Vr_x
            Vi_y = Vt_y + Vr_y

            steer = math.atan2(Vi_y, Vi_x) + WHEEL_ALIGNMENT_OFFSET
            steer = (steer + math.pi) % (2*math.pi) - math.pi

            speed = math.hypot(Vi_x, Vi_y)
            rpm   = (speed / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0
            new_rpm.append(rpm)

            servo_id = SERVO_ID_MAP[i]
            tgt_rad  = self.angle_to_encoder_rad(math.degrees(steer))

            delta = tgt_rad - self.last_pos_rad[i]
            if delta > PERIOD_RAD/2:
                tgt_rad -= PERIOD_RAD
            elif delta < -PERIOD_RAD/2:
                tgt_rad += PERIOD_RAD

            self.publish_steering(servo_id, tgt_rad)
            self.last_pos_rad[i] = tgt_rad

        # Scale down if exceeding MAX_RPM
        m = max(new_rpm)
        if m > MAX_RPM:
            scale = MAX_RPM / m
            new_rpm = [r*scale for r in new_rpm]

        # rotate list right by one to match VESC ordering
        new_rpm = new_rpm[1:] + new_rpm[:1]

        # ramp into new targets
        if any(abs(n - t) > 0.1 for n, t in zip(new_rpm, self.target_rpm)):
            self.ramp_initial_rpm = list(self.current_rpm)
            self.target_rpm       = new_rpm
            self.ramp_start_time  = self.get_clock().now()

    def timer_callback(self):
        if self.ramp_start_time is None:
            self.rpm_pub.publish(Float32MultiArray(data=self.current_rpm))
            return

        elapsed = (self.get_clock().now() - self.ramp_start_time).nanoseconds * 1e-9
        t       = min(max(elapsed / self.ramp_duration, 0.0), 1.0)
        s       = 3*t*t - 2*t*t*t  # smoothstep
        self.current_rpm = [
            r0 + s*(r1 - r0)
            for r0, r1 in zip(self.ramp_initial_rpm, self.target_rpm)
        ]

        self.rpm_pub.publish(Float32MultiArray(data=self.current_rpm))

        if t >= 1.0:
            self.ramp_start_time = None

    # ────── Convenience API ──────
    def full_spin(self, clockwise: bool = True):
        rotation_raw = -8192.0 if clockwise else 8192.0
        self.get_logger().info(
            f"full_spin: {'CW' if clockwise else 'CCW'} at max angular speed"
        )
        msg = Float32MultiArray(data=[0.0, 0.0, rotation_raw])
        self.driving_callback(msg)

    def drive_full_speed(self, angle_deg: float):
        speed_raw    = 8192.0
        rotation_raw = 0.0
        self.get_logger().info(f"drive_full_speed: heading={angle_deg}° at max linear speed")
        msg = Float32MultiArray(data=[angle_deg, speed_raw, rotation_raw])
        self.driving_callback(msg)


class BasketAlignNode(AutoMoveNode):
    def __init__(self):
        super().__init__('basket_align_node')
        # 对齐开关，初始不对齐
        self.align_active = False

        # 订阅 basket_detector/status
        self.create_subscription(
            String,
            '/basket_detector/status',
            self.status_callback,
            10)

        # 订阅 ai_order
        self.create_subscription(
            Float32MultiArray,
            'ai_order',
            self.ai_order_callback,
            10)

    def ai_order_callback(self, msg: Float32MultiArray):
        # 触发对齐：data[0] 从 0 变 1
        if len(msg.data) > 0 and msg.data[0] == 1.0 and not self.align_active:
            self.get_logger().info('[BasketAlign] ai_order triggered → start aligning')
            self.align_active = True

    def status_callback(self, msg: String):
        # 只有在 align_active 时才响应状态
        if not self.align_active:
            return

        status = msg.data.strip()
        self.get_logger().info(f"[BasketAlign] Received status: {status}")

        if status == 'TURN_LEFT':
            self.get_logger().info("[BasketAlign] Hoop is left → rotate CCW")
            self.full_spin(clockwise=False)

        elif status == 'TURN_RIGHT':
            self.get_logger().info("[BasketAlign] Hoop is right → rotate CW")
            self.full_spin(clockwise=True)

        elif status == 'GO_STRAIGHT':
            self.get_logger().info("[BasketAlign] Hoop centered → alignment done")
            # 对齐完成，重置开关
            self.align_active = False

        else:
            self.get_logger().warning(f"[BasketAlign] Unknown status: '{status}'")


def main(args=None):
    rclpy.init(args=args)
    node = BasketAlignNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
