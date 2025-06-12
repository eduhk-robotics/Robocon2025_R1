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

class ActiveCasterNode(Node):
    def __init__(self):
        super().__init__('active_caster_node')
        self.get_logger().info('ActiveCasterNode starting')

        # Last commanded steering positions (rad) for each motor
        self.last_pos_rad = [0.0] * 4  # 4 个 damiao 的最后角度

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

        # VESC RPM publisher (single topic)
        self.rpm_pub = self.create_publisher(
            Float32, 'vesc_control', 10
        )

        # Subscribe to driving commands [heading_deg, speed, rotation_speed]
        self.create_subscription(
            Float32MultiArray,
            'driving',
            self.driving_callback,
            10
        )

        # 定时器发布平滑速度（50 Hz）
        self.create_timer(0.02, self.timer_callback)

    def publish_steering(self, motor_id, target_rad):
        """Publish steering command to damiao_control for a specific wheel."""
        out = Float32MultiArray()
        out.data = [float(motor_id), 2.0, 20.0, target_rad]
        self.pos_pub.publish(out)
        self.get_logger().debug(f"Motor {motor_id}: Steering {target_rad:.2f} rad")

    def publish_rpm(self, rpm_value: float):
        """Publish RPM command to vesc_control."""
        rpm_msg = Float32()
        rpm_msg.data = float(rpm_value)
        self.rpm_pub.publish(rpm_msg)
        self.get_logger().debug(f"VESC: RPM {rpm_value:.1f}")

    def compute_target_rad(self, angle_deg):
        """Convert angle (degrees) to encoder radians."""
        wheel_rev = angle_deg / 360.0
        motor_rev = wheel_rev / GEAR_RATIO
        encoder_rev = motor_rev * ENCODER_RATIO
        return encoder_rev * TWO_PI

    def driving_callback(self, msg: Float32MultiArray):
        direction_deg = float(msg.data[0])  # 手柄输入方向 (度)
        raw_speed = float(msg.data[1])     # 平面速度 [0, 8192]
        rotation_speed = float(msg.data[2])  # 旋转速度 [-8192, 8192]

        rotation_norm = rotation_speed / 8192.0  # 归一化到 [-1, 1]
        rotation_deadzone = 20.0 / 180.0  # ±20° 死区，归一化约 0.111

        if abs(rotation_norm) > rotation_deadzone:
            # 自转模式：右摇杆触发
            if rotation_norm < 0:
                # 逆时针：左推 (rotation_speed < 0)
                angles = [0.0, -90.0, 180.0, 90.0]  # damiao 1,2,3,4
            else:
                # 顺时针：右推 (rotation_speed > 0)
                angles = [0.0, 90.0, -180.0, -90.0]

            # 为每个 damiao 设置角度
            for i, angle_deg in enumerate(angles, start=1):
                target_rad = self.compute_target_rad(angle_deg)
                # 最短运动策略
                delta = target_rad - self.last_pos_rad[i - 1]
                if delta > PERIOD_RAD / 2.0:
                    target_rad -= PERIOD_RAD
                elif delta < -PERIOD_RAD / 2.0:
                    target_rad += PERIOD_RAD
                self.publish_steering(i, target_rad)
                self.last_pos_rad[i - 1] = target_rad
                self.get_logger().info(f"Motor {i}: Steering {angle_deg:.1f}° → {target_rad:.2f} rad")

            # VESC RPM：基于 rotation_speed 绝对值，符号决定方向
            new_target_rpm = rotation_norm * MAX_RPM  # ±9000，负值顺时针
        else:
            # 平移模式：恢复原始逻辑
            target_rad = self.compute_target_rad(direction_deg)
            # 最短运动策略
            delta = target_rad - self.last_pos_rad[0]
            if delta > PERIOD_RAD / 2.0:
                target_rad -= PERIOD_RAD
            elif delta < -PERIOD_RAD / 2.0:
                target_rad += PERIOD_RAD

            # 所有 damiao 设置统一角度
            for i in range(1, 5):
                self.publish_steering(i, target_rad)
                self.last_pos_rad[i - 1] = target_rad
            self.get_logger().info(f"Steering {direction_deg:.1f}° → {target_rad:.2f} rad")

            # VESC RPM：基于 plane_speed
            norm = raw_speed / 8192.0
            new_target_rpm = norm * MAX_RPM  # 0 到 9000

        # 如果目标 RPM 变化，则重置插值
        if abs(new_target_rpm - self.target_rpm) > 1e-1:
            self.ramp_initial_rpm = self.current_rpm
            self.target_rpm = new_target_rpm
            self.ramp_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Start ramp: from {self.ramp_initial_rpm:.1f} to {self.target_rpm:.1f} RPM"
            )

    def timer_callback(self):
        """定时器驱动的平滑插值与发布。"""
        if self.ramp_start_time is None:
            self.publish_rpm(self.current_rpm)
            return

        # 计算插值进度 t
        elapsed = (self.get_clock().now() - self.ramp_start_time).nanoseconds * 1e-9
        t = min(max(elapsed / self.ramp_duration, 0.0), 1.0)

        # 三次插值权重
        s = 3 * t * t - 2 * t * t * t
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