import rclpy
from rclpy.node import Node
from joystick_msgs.msg import Joystick
from std_msgs.msg import Float32

class ShooterControlNode(Node):
    def __init__(self):
        super().__init__('shooter_control_node')
        self.get_logger().info('ShooterControlNode starting')

        # 订阅 joystick 输入
        self.subscription = self.create_subscription(
            Joystick,
            'joystick_input',
            self.joystick_callback,
            10
        )

        # 发布到 shooter_vesc_control 主题
        self.publisher_ = self.create_publisher(Float32, 'shooter_vesc_control', 10)

    def joystick_callback(self, msg):
        """处理手柄右触发器 (rt) 值并映射为 VESC 的 eRPM"""
        try:
            rt_value = msg.rt
        except AttributeError as e:
            self.get_logger().error(f"无效的手柄消息: {e}")
            rt_value = 0

        # 验证 rt 值（预期范围：0 到 255）
        if not (0 <= rt_value <= 255):
            self.get_logger().warn(f"无效的 rt 值: {rt_value}，预期 0 到 255")
            rt_value = max(min(rt_value, 255), 0)

        # 将 rt 从 [0, 255] 映射到 [0, 30000] 作为 eRPM
        erpm = (rt_value / 255.0) * 30000.0

        # 发布到 shooter_vesc_control
        msg = Float32()
        msg.data = float(erpm)
        self.publisher_.publish(msg)

        # 调试日志
        self.get_logger().info(f"收到 rt: {rt_value}，映射后的 eRPM: {erpm:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = ShooterControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
