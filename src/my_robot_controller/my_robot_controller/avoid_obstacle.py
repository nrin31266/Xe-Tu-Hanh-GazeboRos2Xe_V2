#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# ======================= HÀM XỬ LÝ GÓC =======================

def yaw_from_quat(q):
    # Chuyển quaternion (x,y,z,w) sang góc yaw (rad)
    # Yaw là góc quay quanh trục Z, phù hợp robot chạy trên mặt phẳng
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    # Chuẩn hóa góc về [-pi, pi] để robot quay theo hướng ngắn nhất
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    # Giới hạn giá trị v trong [lo, hi] (chống quay quá nhanh)
    return max(lo, min(hi, v))


# ======================= NODE TRÁNH VẬT CẢN =======================

class ObstacleAvoider(Node):
    """
    Node tránh vật cản theo mô hình máy trạng thái (FSM) gồm 4 state:
      1) FORWARD   : đi thẳng và giữ hướng ban đầu (lane_yaw)
      2) TURN_OUT  : gặp vật cản -> quay 90 độ sang trái/phải
      3) PASS      : đi thẳng theo hướng mới một đoạn side_pass_dist
      4) TURN_BACK : quay 90 độ về lại hướng ban đầu rồi tiếp tục FORWARD

    Topic:
      - Sub: /gazebo_ros_laser/out (LaserScan)  : phát hiện vật cản
      - Sub: /odom (Odometry)                  : lấy x,y,yaw + đo quãng đường
      - Pub: /cmd_vel (Twist)                  : điều khiển robot
    """

    def __init__(self):
        super().__init__("obstacle_avoider")

        # Publisher điều khiển robot
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber laser từ Gazebo
        self.scan_sub = self.create_subscription(
            LaserScan, "/gazebo_ros_laser/out", self.scan_cb, 10
        )

        # Subscriber odom
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )

        # ===== SPEED =====
        self.v_fwd = 0.8    # tốc độ đi thẳng bình thường
        self.v_pass = 1.5   # tốc độ khi PASS (né vật cản)
        self.v_turn = 0.0   # quay tại chỗ khi rẽ

        # ===== AVOID DIST =====
        self.stop_dist = 5.0        # phát hiện vật cản phía trước (< stop_dist)
        self.side_pass_dist = 1.4   # đi ngang để vượt vật cản

        # ===== YAW CONTROL =====
        self.k_yaw_turn = 3.6   # gain quay khi rẽ
        self.k_yaw_hold = 1.2   # gain giữ hướng khi đi thẳng
        self.max_w = 1.0        # giới hạn tốc độ góc

        self.yaw_tol = 0.10         # ngưỡng coi như đạt góc
        self.yaw_hold_frames = 1    # số frame đạt liên tiếp
        self.deadband = 0.05        # vùng chết chống rung

        # ===== STATE =====
        self.state = "FORWARD"
        self.turn_dir = 1.0  # +1 LEFT, -1 RIGHT

        # ===== ODOM DATA =====
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ===== YAW TARGETS =====
        self.lane_yaw = 0.0     # hướng ban đầu (giữ khi FORWARD)
        self.target_yaw = 0.0   # mục tiêu khi quay
        self.pass_yaw = 0.0     # hướng khi PASS

        # ===== PASS START =====
        self.pass_start_x = 0.0
        self.pass_start_y = 0.0

        self._yaw_ok_count = 0

        self.get_logger().info("ObstacleAvoider(ODOM) started")
        self.get_logger().info("STATE: FORWARD")  # in state ban đầu

    # In state chỉ khi đổi (không spam terminal)
    def set_state(self, new_state: str):
        if new_state == self.state:
            return
        dir_str = "LEFT" if self.turn_dir > 0.0 else "RIGHT"
        self.get_logger().info(f"STATE: {self.state} -> {new_state} | TURN={dir_str}")
        self.state = new_state

    # Callback odom: cập nhật x, y, yaw
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

        # lần đầu có odom thì chốt hướng ban đầu làm lane_yaw
        if not self.have_odom:
            self.lane_yaw = self.yaw
            self.have_odom = True

    # Lấy khoảng cách nhỏ nhất trong vùng góc [deg_min, deg_max]
    # Dùng để lấy vật cản gần nhất theo từng vùng: front/left/right
    def sector_min(self, msg, deg_min, deg_max):
        a_min = math.radians(deg_min)
        a_max = math.radians(deg_max)

        if msg.angle_increment == 0.0:
            return float("inf")

        i_min = int((a_min - msg.angle_min) / msg.angle_increment)
        i_max = int((a_max - msg.angle_min) / msg.angle_increment)

        # giới hạn index để không bị out of range
        i_min = max(0, min(i_min, len(msg.ranges) - 1))
        i_max = max(0, min(i_max, len(msg.ranges) - 1))

        lo, hi = min(i_min, i_max), max(i_min, i_max)

        # lấy khoảng cách nhỏ nhất trong đoạn [lo..hi]
        m = float("inf")
        for r in msg.ranges[lo: hi + 1]:
            if r is None:
                continue
            if math.isfinite(r) and r > max(0.05, msg.range_min):
                m = min(m, r)

        return m

    # Lưu điểm bắt đầu PASS (để tính quãng đường đã đi)
    def set_pass_start(self):
        self.pass_start_x = self.x
        self.pass_start_y = self.y

    # Tính quãng đường đi từ lúc bắt đầu PASS
    def passed_distance(self):
        dx = self.x - self.pass_start_x
        dy = self.y - self.pass_start_y
        return math.sqrt(dx * dx + dy * dy)

    # Sai số góc target - yaw hiện tại (đã wrap)
    def yaw_error(self, target):
        return wrap_pi(target - self.yaw)

    # Điều khiển quay khi rẽ (TURN_OUT / TURN_BACK)
    def yaw_control_turn(self, target):
        err = self.yaw_error(target)
        if abs(err) < self.deadband:
            return 0.0, err
        w = clamp(self.k_yaw_turn * err, -self.max_w, self.max_w)
        return w, err

    # Giữ hướng khi đi thẳng (FORWARD / PASS)
    def yaw_control_hold(self, target):
        err = self.yaw_error(target)
        if abs(err) < self.deadband:
            return 0.0
        return clamp(self.k_yaw_hold * err, -0.8, 0.8)

    # Kiểm tra đạt góc ổn định chưa (đạt đủ yaw_hold_frames frame)
    def yaw_reached_hold(self, err):
        if abs(err) < self.yaw_tol:
            self._yaw_ok_count += 1
        else:
            self._yaw_ok_count = 0
        return self._yaw_ok_count >= self.yaw_hold_frames

    # Callback laser: quyết định state + xuất cmd_vel
    def scan_cb(self, msg: LaserScan):
        if not self.have_odom:
            return

        # ===== Lấy khoảng cách vật cản 3 hướng =====
        # Mở rộng sector để phân biệt trái/phải tốt hơn (ít nhiễu hơn)
        front = self.sector_min(msg, -10, 10)
        left = self.sector_min(msg, 10, 40)
        right = self.sector_min(msg, -40, -10)

        cmd = Twist()

        # ================== STATE 1: FORWARD ==================
        if self.state == "FORWARD":
            # đi thẳng + giữ hướng lane_yaw
            cmd.linear.x = self.v_fwd
            cmd.angular.z = self.yaw_control_hold(self.lane_yaw)

            # Nếu phía trước có vật cản gần -> chọn hướng né và chuyển TURN_OUT
            if front < self.stop_dist:
                # In ra để debug thuật toán rẽ trái/phải
                self.get_logger().info(
                    f"DECIDE: front={front:.2f} left={left:.2f} right={right:.2f}"
                )

                # ===== Quy tắc chọn hướng né =====
                # - rẽ về phía có khoảng trống lớn hơn (giá trị laser lớn hơn)
                # - nếu gần bằng nhau thì GIỮ hướng rẽ trước đó (đỡ bias LEFT)
                eps = 0.05
                if left > right + eps:
                    self.turn_dir = 1.0
                elif right > left + eps:
                    self.turn_dir = -1.0
                # else: giữ self.turn_dir cũ

                # Góc mục tiêu quay 90 độ so với hướng ban đầu
                self.target_yaw = wrap_pi(
                    self.lane_yaw + self.turn_dir * (math.pi / 2.0)
                )
                self.set_state("TURN_OUT")

        # ================== STATE 2: TURN_OUT ==================
        elif self.state == "TURN_OUT":
            # quay tại chỗ để đạt target_yaw
            cmd.linear.x = self.v_turn
            w, err = self.yaw_control_turn(self.target_yaw)
            cmd.angular.z = w

            # Quay đủ góc -> lưu điểm bắt đầu PASS
            if abs(err) < 0.14 or self.yaw_reached_hold(err):
                self.set_pass_start()
                self.pass_yaw = self.yaw
                self.set_state("PASS")

        # ================== STATE 3: PASS ==================
        elif self.state == "PASS":
            # đi nhanh theo hướng pass_yaw để vượt vật cản
            cmd.linear.x = self.v_pass
            cmd.angular.z = self.yaw_control_hold(self.pass_yaw)

            # Đi đủ quãng đường né -> quay về hướng ban đầu
            if self.passed_distance() >= self.side_pass_dist:
                self.target_yaw = wrap_pi(
                    self.pass_yaw - self.turn_dir * (math.pi / 2.0)
                )
                self._yaw_ok_count = 0
                self.set_state("TURN_BACK")

        # ================== STATE 4: TURN_BACK ==================
        elif self.state == "TURN_BACK":
            # quay tại chỗ về lại hướng lane
            cmd.linear.x = self.v_turn
            w, err = self.yaw_control_turn(self.target_yaw)
            cmd.angular.z = w

            # Quay đủ góc -> trở về FORWARD
            if self.yaw_reached_hold(err) or abs(err) < 0.12:
                self.set_state("FORWARD")

        # Publish lệnh điều khiển cho robot
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

