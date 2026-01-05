#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# HAM DE TINH YAW TU QUATERNION
def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# HAM GIU GOC YAW TRONG KHOANG -PI DEN PI
def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

# HAM GIU GIA TRI TRONG KHOANG [LO, HI]
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)  # PUBLISH CMD_VEL TO MOVE THE ROBOT
        self.scan_sub = self.create_subscription(LaserScan, "/gazebo_ros_laser/out", self.scan_cb, 10)  # SUBSCRIBE LASER SCAN
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)  # SUBSCRIBE ODOMETRY: GET POSITION + YAW(HEADING)
        self.finish_sub = self.create_subscription(Bool, "/race_finished", self.finish_cb, 10)  # SUBSCRIBE FINISH FLAG

        # SPEED
        self.v_fwd = 0.8
        self.v_pass = 1.5
        self.v_turn = 0.0

        # AVOID DIST
        self.stop_dist = 5.0
        self.side_pass_dist = 1.4  # DIST TO PASS OBSTACLE ON SIDE

        # YAW CONTROL
        self.k_yaw_turn = 3.6
        self.k_yaw_hold = 1.2
        self.max_w = 1.0
        self.yaw_tol = 0.10
        self.yaw_hold_frames = 1
        self.deadband = 0.05

        # LANE LIMIT (ODOM Y)
        self.lane_center_y = 0.0
        self.lane_half_width = 2.00 # BAN RONG LANE
        self.lane_margin = 0.25    # BIEN CACH LANE
        self.side_pass_target = self.side_pass_dist

        # STATE
        self.state = "FORWARD"
        self.turn_dir = 1.0  # +1 LEFT, -1 RIGHT

        # ODOM DATA
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # VAN TOC THUC TE DE PHANH
        self.vx = 0.0
        self.wz = 0.0

        self.lane_yaw = 0.0
        self.target_yaw = 0.0
        self.pass_yaw = 0.0
        self.pass_start_x = 0.0
        self.pass_start_y = 0.0
        self._yaw_ok_count = 0

        # FINISH + BRAKE
        self.finished = False
        self.brake_timer = None
        self.brake_phase = "NONE"
        self.brake_start_time = None

        self.get_logger().info("START")
        self.get_logger().info("STATE FORWARD")

    def log_state(self, new_state: str):
        if new_state == self.state:
            return
        d = "LEFT" if self.turn_dir > 0.0 else "RIGHT"
        self.get_logger().info(f"STATE {self.state} -> {new_state} DIR {d}")
        self.state = new_state

    # TINH KHOANG TRONG CON LAI DEN BIEN LANE THEO HUONG RE
    def lane_room(self, turn_dir: float):
        # turn_dir = +1 (LEFT) => y tang
        # turn_dir = -1 (RIGHT) => y giam
        y_max = self.lane_center_y + (self.lane_half_width - self.lane_margin)
        y_min = self.lane_center_y - (self.lane_half_width - self.lane_margin)

        if turn_dir > 0.0:
            return y_max - self.y
        else:
            return self.y - y_min

    def finish_cb(self, msg: Bool):
        if bool(msg.data) and not self.finished:
            self.finished = True
            self._yaw_ok_count = 0

            self.brake_phase = "REVERSE_BRAKE"
            self.brake_start_time = self.get_clock().now()

            self.get_logger().warn("FINISH -> BRAKE LOCK")

            self.publish_brake()
            if self.brake_timer is None:
                self.brake_timer = self.create_timer(0.01, self.publish_brake)

    def publish_brake(self):
        if not self.finished:
            return

        cmd = Twist()
        t = (self.get_clock().now() - self.brake_start_time).nanoseconds * 1e-9 if self.brake_start_time else 0.0

        v_eps = 0.03
        w_eps = 0.05

        if self.brake_phase == "REVERSE_BRAKE":
            if abs(self.vx) < v_eps and abs(self.wz) < w_eps:
                self.brake_phase = "HOLD_ZERO"

            if t > 0.25:
                self.brake_phase = "HOLD_ZERO"

            if self.brake_phase == "REVERSE_BRAKE":
                cmd.linear.x = clamp(-1.2 * self.vx, -0.8, 0.8)
                cmd.angular.z = clamp(-1.5 * self.wz, -1.0, 1.0)

        if self.brake_phase == "HOLD_ZERO":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

        if not self.have_odom:
            self.lane_yaw = self.yaw
            self.lane_center_y = self.y
            self.have_odom = True
            self.get_logger().info("ODOM RECEIVED -> READY")

    def sector_min(self, msg, deg_min, deg_max):
        a_min = math.radians(deg_min)
        a_max = math.radians(deg_max)
        if msg.angle_increment == 0.0:
            return float("inf")

        i_min = int((a_min - msg.angle_min) / msg.angle_increment)
        i_max = int((a_max - msg.angle_min) / msg.angle_increment)
        i_min = max(0, min(i_min, len(msg.ranges) - 1))
        i_max = max(0, min(i_max, len(msg.ranges) - 1))
        lo, hi = min(i_min, i_max), max(i_min, i_max)

        m = float("inf")
        for r in msg.ranges[lo: hi + 1]:
            if r is None:
                continue
            if math.isfinite(r) and r > max(0.05, msg.range_min):
                m = min(m, r)
        return m

    def set_pass_start(self):
        self.pass_start_x = self.x
        self.pass_start_y = self.y

    def passed_distance(self):
        dx = self.x - self.pass_start_x
        dy = self.y - self.pass_start_y
        return math.sqrt(dx * dx + dy * dy)

    def yaw_error(self, target):
        return wrap_pi(target - self.yaw)

    def yaw_control_turn(self, target):
        err = self.yaw_error(target)
        if abs(err) < self.deadband:
            return 0.0, err
        w = clamp(self.k_yaw_turn * err, -self.max_w, self.max_w)
        return w, err

    def yaw_control_hold(self, target):
        err = self.yaw_error(target)
        if abs(err) < self.deadband:
            return 0.0
        return clamp(self.k_yaw_hold * err, -0.8, 0.8)

    def yaw_reached_hold(self, err):
        if abs(err) < self.yaw_tol:
            self._yaw_ok_count += 1
        else:
            self._yaw_ok_count = 0
        return self._yaw_ok_count >= self.yaw_hold_frames

    def scan_cb(self, msg: LaserScan):
        if not self.have_odom:
            return
        if self.finished:
            return

        front = self.sector_min(msg, -10, 10)
        left = self.sector_min(msg, 10, 40)
        right = self.sector_min(msg, -40, -10)

        cmd = Twist()

        if self.state == "FORWARD":
            cmd.linear.x = self.v_fwd
            cmd.angular.z = self.yaw_control_hold(self.lane_yaw)

            if front < self.stop_dist:
                self.get_logger().info(f"OBS [FRONT: {front:.2f}, L: {left:.2f}, R: {right:.2f}]")

                # CHON HUONG THEO KHOANG TRONG
                eps = 0.05
                cand = self.turn_dir
                if left > right + eps:
                    cand = 1.0
                elif right > left + eps:
                    cand = -1.0

                # CHAN VUOT BIEN LANE
                room_left = self.lane_room(1.0)
                room_right = self.lane_room(-1.0)

                # NEU HUONG DU DINH KHONG CON CHO -> DOI HUONG
                if cand > 0.0 and room_left < self.lane_margin:
                    cand = -1.0
                    self.get_logger().info("LANE FORCE RIGHT")
                elif cand < 0.0 and room_right < self.lane_margin:
                    cand = 1.0
                    self.get_logger().info("LANE FORCE LEFT")

                self.turn_dir = cand

                # TINH PASS TARGET THEO CHO TRONG BIEN
                room = max(0.0, self.lane_room(self.turn_dir))
                self.side_pass_target = min(self.side_pass_dist, max(0.20, room))
                self.get_logger().info(f"PASS TARGET {self.side_pass_target:.2f}")

                self.target_yaw = wrap_pi(self.lane_yaw + self.turn_dir * (math.pi / 2.0))
                self.log_state("TURN_OUT")

        elif self.state == "TURN_OUT":
            cmd.linear.x = self.v_turn
            w, err = self.yaw_control_turn(self.target_yaw)
            cmd.angular.z = w
            if abs(err) < 0.14 or self.yaw_reached_hold(err):
                self.set_pass_start()
                self.pass_yaw = self.yaw
                self.log_state("PASS")

        elif self.state == "PASS":
            cmd.linear.x = self.v_pass
            cmd.angular.z = self.yaw_control_hold(self.pass_yaw)

            # NEU SAP VUOT BIEN -> QUAY LAI NGAY
            if self.lane_room(self.turn_dir) < self.lane_margin:
                self.get_logger().info("LANE EDGE -> TURN BACK")
                self.target_yaw = wrap_pi(self.pass_yaw - self.turn_dir * (math.pi / 2.0))
                self._yaw_ok_count = 0
                self.log_state("TURN_BACK")
            else:
                if self.passed_distance() >= self.side_pass_target:
                    self.target_yaw = wrap_pi(self.pass_yaw - self.turn_dir * (math.pi / 2.0))
                    self._yaw_ok_count = 0
                    self.log_state("TURN_BACK")

        elif self.state == "TURN_BACK":
            cmd.linear.x = self.v_turn
            w, err = self.yaw_control_turn(self.target_yaw)
            cmd.angular.z = w
            if self.yaw_reached_hold(err) or abs(err) < 0.12:
                self.log_state("FORWARD")

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
