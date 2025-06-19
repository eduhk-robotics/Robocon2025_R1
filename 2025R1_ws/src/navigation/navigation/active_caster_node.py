
#!/usr/bin/env python3
"""
ActiveCasterNode (Fixed with hardware servo ID mapping)
Swerve-drive kinematics with simultaneous translation and rotation.

Publishes:
    - damiao_control (Float32MultiArray) - steering angle commands
    - vesc_control   (Float32MultiArray) - per-wheel RPM commands
Subscribes:
    - driving (Float32MultiArray) - [heading_deg, speed_raw, rotation_raw]
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Encoder / gear constants
ENCODER_RATIO = 19.20321         # encoder counts per motor revolution
GEAR_RATIO    = 61.0 / 35.0      # wheel_rev = motor_rev / GEAR_RATIO
TWO_PI        = 2.0 * math.pi
# encoder_rad per wheel revolution = TWO_PI * ENCODER_RATIO * GEAR_RATIO
PERIOD_RAD    = TWO_PI * ENCODER_RATIO * GEAR_RATIO

# Geometry and speed limits
DIAMOND_RADIUS   = 0.267552      # m, distance from center in diamond layout
WHEEL_RADIUS     = 0.05          # m, wheel radius

# Alignment offset (adjust sign or value if needed)
WHEEL_ALIGNMENT_OFFSET = math.pi / 2

# Hardware servo ID mapping
#  WHEEL_POS index: 0=right, 1=rear, 2=left, 3=front
#  Desired servo IDs: right→1, rear→2, left→3, front→4
SERVO_ID_MAP = [1, 2, 3, 4]

# Diamond layout positions
WHEEL_POS = [
    ( DIAMOND_RADIUS,   0.0),  # index 0: right
    ( 0.0,             -DIAMOND_RADIUS),  # index 1: rear
    (-DIAMOND_RADIUS,   0.0),  # index 2: left
    ( 0.0,              DIAMOND_RADIUS),  # index 3: front
]

MAX_LINEAR_MPS  = 2.0
MAX_ANGULAR_RPS = math.pi
MAX_RPM         = 9000

class ActiveCasterNode(Node):
    def __init__(self):
        super().__init__('active_caster_node')
        self.get_logger().info('ActiveCasterNode starting')

        # Steering state
        self.last_pos_rad     = [0.0] * 4

        # RPM ramp state
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

    def driving_callback(self, msg):
        heading_deg  = float(msg.data[0])
        speed_raw    = float(msg.data[1])
        rotation_raw = float(msg.data[2])

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
            # rotational component
            Vr_x = -omega * ry
            Vr_y =  omega * rx
            # combined velocity
            Vi_x = Vt_x + Vr_x
            Vi_y = Vt_y + Vr_y

            # steering angle tangent + offset
            steer = math.atan2(Vi_y, Vi_x) + WHEEL_ALIGNMENT_OFFSET
            steer = (steer + math.pi) % (2*math.pi) - math.pi

            # wheel speed (rpm)
            speed = math.hypot(Vi_x, Vi_y)
            rpm   = (speed / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0
            new_rpm_targets.append(rpm)

            # map to hardware servo ID
            servo_id = SERVO_ID_MAP[i-1]
            tgt_rad  = self.angle_to_encoder_rad(math.degrees(steer))

            # shortest path adjustment
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

        # ramp into new targets if changed
        if any(abs(n - t) > 1e-1 for n, t in zip(new_rpm_targets, self.target_rpm)):
            self.ramp_initial_rpm = list(self.current_rpm)
            self.target_rpm       = list(new_rpm_targets)
            self.ramp_start_time  = self.get_clock().now()

    def timer_callback(self):
        if self.ramp_start_time is None:
            msg = Float32MultiArray()
            msg.data = self.current_rpm
            self.rpm_pub.publish(msg)
            return

        elapsed = (self.get_clock().now() - self.ramp_start_time).nanoseconds * 1e-9
        t       = min(max(elapsed / self.ramp_duration, 0.0), 1.0)
        s       = 3*t*t - 2*t*t*t  # smoothstep
        self.current_rpm = [
            r0 + s*(r1 - r0)
            for r0, r1 in zip(self.ramp_initial_rpm, self.target_rpm)
        ]

        msg = Float32MultiArray()
        msg.data = self.current_rpm
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