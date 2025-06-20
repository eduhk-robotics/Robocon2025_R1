import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from std_msgs.msg import Float32MultiArray
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'driving', 10)

        # 死区阈值（normalized 0–1）
        self.deadzone = 0.2

        # 存储上一次非零方向，用于“无输入时保持方向”
        self.last_direction = 0.0
        # 存储上一次非零旋转速度，用于“无输入时保持旋转”
        self.last_rotation_speed = 0.0

        # 声明参数
        self.declare_parameter('dpad_erpm', 1300.0)
        self.declare_parameter('max_rpm', 10000.0)

        dpad_erpm = self.get_parameter('dpad_erpm').value
        max_rpm = self.get_parameter('max_rpm').value
        self.dpad_fixed_speed = (dpad_erpm / max_rpm) * 8192.0

    def map_value(self, value):
        """Maps joystick input from [-32768, 32767] to [-1, 1]."""
        return ((value + 32768) / 65535) * 2.0 - 1.0

    def listener_callback(self, msg):
        # 读取原始输入
        try:
            raw_lx = msg.lx
            raw_ly = msg.ly
            raw_rx = msg.rx
            dpad_x = msg.dx
            dpad_y = msg.dy
        except AttributeError as e:
            self.get_logger().error(f"Invalid Joystick message: {e}")
            return

        # 检查 D-pad 值范围
        if dpad_x not in (-1, 0, 1) or dpad_y not in (-1, 0, 1):
            self.get_logger().warn(f"Invalid D-pad input: dx={dpad_x}, dy={dpad_y}")

        # —— 圆形死区处理：先归一化，再整体判断 —— #
        # 左摇杆
        nx = self.map_value(raw_lx)
        ny = self.map_value(raw_ly)
        mag = math.hypot(nx, ny)
        if mag < self.deadzone:
            left_x = 0.0
            left_y = 0.0
        else:
            left_x = nx
            left_y = ny

        # 右摇杆（旋转）
        nr = self.map_value(raw_rx)
        right_x = nr if abs(nr) >= self.deadzone else 0.0

        # 初始化输出
        direction = self.last_direction
        plane_speed = 0.0
        control_source = "none"

        # 主控：左摇杆
        if left_x != 0.0 or left_y != 0.0:
            # 计算航向角（北=0度，顺时针增）
            angle = math.degrees(math.atan2(left_y, left_x))
            if angle < 0.0:
                angle += 360.0
            direction = (angle + 90.0) % 360.0
            # 速度：向量长度映射到 [0,8192]
            speed = min(mag, 1.0) * 8192.0
            plane_speed = speed
            self.last_direction = direction
            control_source = "joystick"

        # 备控：D-pad
        elif dpad_x != 0 or dpad_y != 0:
            if dpad_y == -1:
                direction = 0.0
            elif dpad_y == 1:
                direction = 180.0
            elif dpad_x == -1:
                direction = 270.0
            elif dpad_x == 1:
                direction = 90.0
            plane_speed = self.dpad_fixed_speed
            self.last_direction = direction
            control_source = "dpad"

        # 计算旋转速度
        rotation_speed = -right_x * 8192.0
        # 限幅
        rotation_speed = max(min(rotation_speed, 8192.0), -8192.0)
        # 当有旋转输入时，更新 last_rotation_speed
        if right_x != 0.0:
            self.last_rotation_speed = rotation_speed

        # —— 如果没有任何输入（直行+旋转都为零），则发布最后一次：方向不变、线速度置0、旋转速度保持 —— #
        if plane_speed == 0.0 and rotation_speed == 0.0:
            driving_msg = Float32MultiArray()
            driving_msg.data = [
                float(direction),            # 保持上次方向
                114514520.0,                         # 线速度置0
                float(self.last_rotation_speed)  # 旋转速度保持
            ]
            self.publisher_.publish(driving_msg)
            return 

        # —— 正常发布消息 —— #
        driving_msg = Float32MultiArray()
        driving_msg.data = [
            float(direction),
            float(plane_speed),
            float(rotation_speed)
        ]
        self.publisher_.publish(driving_msg)

        # 调试日志
        self.get_logger().info(
            f"Control: {control_source}, "
            f"Raw: lx={raw_lx}, ly={raw_ly}, rx={raw_rx}, dx={dpad_x}, dy={dpad_y}, "
            f"Processed: lx={left_x:.2f}, ly={left_y:.2f}, rx={right_x:.2f}, "
            f"Driving: dir={direction:.1f} deg, lin={plane_speed:.1f}, rot={rotation_speed:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
