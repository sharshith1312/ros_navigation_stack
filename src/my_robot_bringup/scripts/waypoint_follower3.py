# gazebo_waypoint_nav/navigator.py
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
                continue  # Skip empty lines and comments
            parts = list(map(float, line.split()))
            if len(parts) == 3:
                waypoints.append(parts)
    print("Way POints reading done...")
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

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def control_loop(self):
        if self.pose is None or self.current_index >= len(self.waypoints):
            return

        x, y, theta_target = self.waypoints[self.current_index]
        pos = self.pose.position

        dx = x - pos.x
        dy = y - pos.y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - self.yaw), math.cos(target_angle - self.yaw))

        cmd = Twist()
        if abs(angle_diff) > 0.1:
            cmd.angular.z = -1.0 * angle_diff
            # 0.5 to 1
        elif distance > 0.1:
            cmd.linear.x = -0.5
            # 0.2 for slow
        else:
            self.get_logger().info(f"Reached waypoint {self.current_index+1}")
            self.current_index += 1

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


main()