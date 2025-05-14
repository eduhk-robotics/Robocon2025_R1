#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(Joy, 'ps4', self.listener_callback, 10)
        self.publisher_  = self.create_publisher(Float32MultiArray, 'driving', 10)
        self.deadzone    = 0.1

    def apply_deadzone(self, value):
        return value if abs(value) >= self.deadzone else 0.0

    def listener_callback(self, msg):
        # ����ҡ������
        left_x  = self.apply_deadzone(msg.axes[0])
        left_y  = self.apply_deadzone(msg.axes[1])
        right_x = self.apply_deadzone(msg.axes[3])

        # ����Ǽ���
        direction = math.degrees(math.atan2(left_y, left_x))
        if direction < 0.0:
            direction += 360.0
        direction = (direction + 90.0) % 360.0
        if direction == 90.0 and left_x == 0.0 and left_y == 0.0:
            direction = 0.0

        # ƽ���ٶ�
        plane_speed = math.hypot(left_x, left_y) * 8192.0
        plane_speed = min(plane_speed, 8192.0)

        # ��ת�ٶ�
        rotation_speed = -right_x * 8192.0
        rotation_speed = max(min(rotation_speed, 8192.0), -8192.0)

        # ��װ��Ϣ
        driving_msg = Float32MultiArray()
        driving_msg.data = [
            float(direction),
            float(plane_speed),
            float(rotation_speed),
        ]

        # �������ƶ�����תʱ����
        if plane_speed > 0.0 or abs(right_x) > 0.0:
            self.get_logger().info(
                f"Publishing driving: dir={direction:.1f} deg, "
                f"lin={plane_speed:.1f}, rot={rotation_speed:.1f}"
            )
            self.publisher_.publish(driving_msg)

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
