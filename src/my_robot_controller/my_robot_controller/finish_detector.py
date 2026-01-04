import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FinishDetector(Node):
    def __init__(self):
        super().__init__('finish_detector')

        self.finish_x = 48.0
        self.reached = False

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer sẽ được tạo khi tới đích để giữ lệnh stop liên tục
        self.stop_timer = None

        self.get_logger().info('🚗 Finish detector started...')

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x

        if x >= self.finish_x and not self.reached:
            self.reached = True
            self.get_logger().info('🎉🎉🎉 ĐÃ ĐẾN ĐÍCH - ĐANG DỪNG XE 🎉🎉🎉')

            # Publish stop ngay lập tức
            self.publish_stop()

            # Và giữ stop liên tục (20Hz) để không bị node khác ghi đè
            self.stop_timer = self.create_timer(0.05, self.publish_stop)


def main(args=None):
    rclpy.init(args=args)
    node = FinishDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

