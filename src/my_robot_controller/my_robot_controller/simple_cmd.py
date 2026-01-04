import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_cmd_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.3   # tốc độ tiến (m/s)
        msg.angular.z = 0.0  # không quay
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
