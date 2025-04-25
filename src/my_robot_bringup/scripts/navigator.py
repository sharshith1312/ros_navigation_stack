import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = list(map(float, line.split()))
            if len(parts) == 2:
                waypoints.append((parts[0], parts[1]))
    print("Waypoints loaded:", waypoints)
    return waypoints

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pose = None
        self.yaw = 0.0
        self.waypoints = read_waypoints('./waypoints1.txt')
        self.current_index = 0
        self.initial_position_printed = False

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        if not self.initial_position_printed:
            print(f"[Initial Pose] x: {self.pose.position.x:.2f}, y: {self.pose.position.y:.2f}, yaw: {self.yaw:.2f}")
            self.initial_position_printed = True

    def control_loop(self):
        if self.pose is None or self.current_index >= len(self.waypoints):
            return

        pos = self.pose.position

        x, y = self.waypoints[self.current_index]

        dx = x - pos.x
        dy = y - pos.y
        distance = math.hypot(dx, dy)

        # Heading to waypoint
        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - self.yaw), math.cos(target_angle - self.yaw))
        # Fix sudden flips near -pi/+pi
        if abs(abs(angle_diff) - math.pi) < 0.2:
            print("Caught near-π wraparound, suppressing turn")
            angle_diff = 0.0

        print(f"[Waypoint {self.current_index + 1}] Distance: {distance:.2f}, "
              f"Yaw: {self.yaw:.2f}, Target Angle: {target_angle:.2f}, Diff Angle: {angle_diff:.2f}")

        cmd = Twist()

        if distance > 0.5:
            cmd.linear.x = -0.2
            cmd.angular.z = -0.8 * angle_diff  # live correction
        else:
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}")
            self.get_logger().info(
                f"   Robot Position → x: {pos.x:.2f}, y: {pos.y:.2f}, yaw: {self.yaw:.2f}"
            )
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.publisher_.publish(stop_cmd)  # Stop
            self.current_index += 1
            return

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()
