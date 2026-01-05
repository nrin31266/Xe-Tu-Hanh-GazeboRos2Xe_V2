import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class FinishDetector(Node):
    def __init__(self):
        super().__init__('finish_detector')

        self.finish_x = 70.0  # X COORDINATE OF FINISH LINE
        self.reached = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.finish_pub = self.create_publisher(Bool, '/race_finished', 10)

        self.stop_timer = None
        self.get_logger().info('Finish detector started...')

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x

        if x >= self.finish_x and not self.reached:
            self.reached = True
            self.get_logger().info('🏁 FINISH LINE REACHED')

            flag = Bool()
            flag.data = True
            for _ in range(10):  # bắn vài phát cho chắc
                self.finish_pub.publish(flag)

            # vẫn publish stop để hỗ trợ (không bắt buộc)
            self.publish_stop()
            self.stop_timer = self.create_timer(0.05, self.publish_stop)

def main(args=None):
    rclpy.init(args=args)
    node = FinishDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
