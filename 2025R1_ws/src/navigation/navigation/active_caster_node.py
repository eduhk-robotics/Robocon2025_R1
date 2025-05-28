import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

# Steering encoder/gear constants
ENCODER_RATIO = 19.20321
GEAR_RATIO = 35.0 / 61.0
TWO_PI = 2.0 * math.pi
PERIOD_RAD = TWO_PI * (ENCODER_RATIO / GEAR_RATIO)  # encoder rad per full wheel turn

# Motor speed constants
MAX_RPM = 9000  # 最大速度（VESC最大电机转速）

class OmniWheelMotorController(Node):
    def __init__(self):
        super().__init__('omni_wheel_motor_controller')
        self.get_logger().info('OmniWheelMotorController starting')

        # Last commanded steering position (rad)
        self.last_pos_rad = 0.0

        # --- 速度平滑状态 ---
        self.current_rpm = 0.0            # 上次发布的速度
        self.target_rpm = 0.0             # 最新目标速度
        self.ramp_initial_rpm = 0.0       # 本次插值起始速度
        self.ramp_start_time = None       # 本次插值开始时间
        self.ramp_duration = 0.5          # 插值时长 (秒)，可调整

        # Steering command publisher
        self.pos_pub = self.create_publisher(
            Float32MultiArray, 'damiao_control', 10
        )

        # VESC RPM publisher
        self.rpm_pub = self.create_publisher(
            Float32, 'vesc_control', 10
        )

        # Subscribe to driving commands [heading_deg, speed]
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # 定时器发布平滑速度（50 Hz）
        self.create_timer(0.02, self.timer_callback)

    def publish_steering(self, target_rad):
        """Publish steering commands to damiao_control for all 4 wheels."""
        for motor_id in range(1, 5):
            out = Float32MultiArray()
            # 消息内容中 motor_id, 固定参数 2.0 与 20.0，以及转向目标（弧度）
            out.data = [float(motor_id), 2.0, 20.0, target_rad]
            self.pos_pub.publish(out)

    def publish_rpm(self, rpm_value: float):
        """Publish RPM command to vesc_control."""
        rpm_msg = Float32()
        rpm_msg.data = float(rpm_value)
        self.rpm_pub.publish(rpm_msg)

    def driving_callback(self, msg: Float32MultiArray):
        # 1) 转向部分（与原逻辑相同）
        direction_deg = float(msg.data[0])
        raw_speed     = float(msg.data[1])

        # Steering: map heading to encoder rad
        wheel_rev   = direction_deg / 360.0
        motor_rev   = wheel_rev / GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        target_rad  = encoder_rev * TWO_PI

        # 最短运动策略
        delta = target_rad - self.last_pos_rad
        if delta >  PERIOD_RAD / 2.0:
            target_rad -= PERIOD_RAD
        elif delta < -PERIOD_RAD / 2.0:
            target_rad += PERIOD_RAD

        self.publish_steering(target_rad)
        self.last_pos_rad = target_rad
        self.get_logger().info(f"Steering {direction_deg:.1f}° → {target_rad:.2f} rad")

        # 2) 速度部分：先计算原始目标 RPM，然后设置平滑插值参数
        norm = raw_speed / 8192.0
        new_target_rpm = norm * MAX_RPM

        # 如果目标变化，则重置插值
        if abs(new_target_rpm - self.target_rpm) > 1e-1:
            self.ramp_initial_rpm = self.current_rpm
            self.target_rpm       = new_target_rpm
            self.ramp_start_time  = self.get_clock().now()
            self.get_logger().info(
                f"Start ramp: from {self.ramp_initial_rpm:.1f} to {self.target_rpm:.1f} RPM"
            )

    def timer_callback(self):
        """定时器驱动的平滑插值与发布。"""
        if self.ramp_start_time is None:
            # 无需插值，直接发布 last RPM
            self.publish_rpm(self.current_rpm)
            return

        # 计算插值进度 t
        elapsed = (self.get_clock().now() - self.ramp_start_time).nanoseconds * 1e-9
        t = min(max(elapsed / self.ramp_duration, 0.0), 1.0)

        # 三次插值权重
        s = 3*t*t - 2*t*t*t
        interp_rpm = self.ramp_initial_rpm + s * (self.target_rpm - self.ramp_initial_rpm)
        self.current_rpm = interp_rpm

        # 发布当前插值 RPM
        self.publish_rpm(self.current_rpm)

        # 如果已到达或超时，停止插值
        if t >= 1.0:
            self.ramp_start_time = None
            self.get_logger().info(f"Reached target RPM: {self.current_rpm:.1f}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
