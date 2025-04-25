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
        self.timer = self.create_timer(0.2, self.control_loop)
        self.pose = None
        self.yaw = 0.0
        self.waypoints = read_waypoints('./waypoints1.txt')
        self.current_index = 0
        self.target_angle = None
        self.aligning = True
        self.initial_position_printed = False

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        _, _, raw_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        # Fix yaw if robot is visually facing -X
        self.yaw = raw_yaw + math.pi
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi  # Normalize to [-π, π]

        if not self.initial_position_printed:
            print(f"Initial Pose: x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f}, yaw={self.yaw:.2f}")
            self.initial_position_printed = True
        # Debug odometry update rate
        print(f"Odometry update: x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f}, yaw={self.yaw:.2f}")

    def control_loop(self):
        if self.pose is None or self.current_index >= len(self.waypoints):
            return

        pos = self.pose.position
        x_target, y_target = self.waypoints[self.current_index]

        dx = x_target - pos.x
        dy = y_target - pos.y
        distance = math.hypot(dx, dy)

        # Compute target angle once
        if self.target_angle is None:
            self.target_angle = math.atan2(dy, dx)
            print(f"Waypoint {self.current_index + 1}")
            print(f"Target position: ({x_target:.2f}, {y_target:.2f})")
            print(f"Current robot position: ({pos.x:.2f}, {pos.y:.2f})")
            print(f"dx = {dx:.2f}, dy = {dy:.2f}, target_angle = {self.target_angle:.2f}")

        # Calculate normalized angle difference
        raw_diff = self.target_angle - self.yaw
        angle_diff = math.atan2(math.sin(raw_diff), math.cos(raw_diff))
        print(f"Raw diff: {raw_diff:.2f}, Angle diff: {angle_diff:.2f}")

        print(f"Current yaw: {self.yaw:.2f}, Target angle: {self.target_angle:.2f}, Angle diff: {angle_diff:.2f}, Distance: {distance:.2f}")

        cmd = Twist()

        # Step 1: Align before driving
        if self.aligning:
            if abs(angle_diff) > 0.01:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.2 * angle_diff 
                # cmd.angular.z = max(min(cmd.angular.z, 1.5), -1.5)  # Clamp to [-1.5, 1.5]
                print(f"Rotating to face waypoint {self.current_index + 1}, cmd.angular.z: {cmd.angular.z:.2f}")
                self.publisher_.publish(cmd)
                return
            else:
                print("Aligned. Ready to move forward.")
                self.aligning = False

        # Step 2: Drive toward target
        if distance > 0.3:
            cmd.linear.x = -0.5  # Positive for forward motion
            cmd.angular.z = 0.5 * angle_diff
            print("Driving forward...")
        else:
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}")
            self.get_logger().info(f"Final Pose: x={pos.x:.2f}, y={pos.y:.2f}, yaw={self.yaw:.2f}")
            self.publisher_.publish(Twist())  # Stop
            self.current_index += 1
            self.target_angle = None
            self.aligning = True
            return

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()