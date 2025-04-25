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
    print("✅ Waypoints loaded:", waypoints)
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
        self.target_angle = None
        self.rotating_to_next = False
        self.initial_position_printed = False

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        orientation_q = self.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        if not self.initial_position_printed:
            print(f"📍 Initial Pose: x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f}, yaw={self.yaw:.2f}")
            self.initial_position_printed = True

    def control_loop(self):
        if self.pose is None or self.current_index >= len(self.waypoints):
            return

        pos = self.pose.position
        x_target, y_target = self.waypoints[self.current_index]

        dx = x_target - pos.x
        dy = y_target - pos.y
        distance = math.hypot(dx, dy)

        # Compute target angle ONCE per waypoint
        if self.target_angle is None:
            self.target_angle = math.atan2(dy, dx)
            print(f"\n➡️ Waypoint {self.current_index + 1}")
            print(f"Target position: ({x_target:.2f}, {y_target:.2f})")
            print(f"Current robot position: ({pos.x:.2f}, {pos.y:.2f})")
            print(f"dx = {dx:.2f}, dy = {dy:.2f}, target_angle = {self.target_angle:.2f}")

        # Compute angle difference (normalized between -π and +π)
        raw_diff = self.target_angle - self.yaw
        angle_diff = math.atan2(math.sin(raw_diff), math.cos(raw_diff))

        # Handle wraparound issue
        if abs(abs(angle_diff) - math.pi) < 0.05:
            print("⚠️ Near-pi wraparound suppressed")
            angle_diff = 0.0


        print(f"Current yaw: {self.yaw:.2f}, Angle diff: {angle_diff:.2f}, Distance: {distance:.2f}")

        cmd = Twist()

        # Handle post-waypoint reorientation (rotation only)
        if self.rotating_to_next:
            if abs(angle_diff) > 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = -1.3 * angle_diff
                print("🔄 Rotating to face next waypoint...")
                self.publisher_.publish(cmd)
                return
            else:
                print("✅ Rotation complete, resuming normal navigation.")
                self.rotating_to_next = False

        # Main navigation logic
        if distance > 0.3:
            if abs(angle_diff) > 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = -1.3 * angle_diff
                print("🔄 Rotating in place...")
            else:
                cmd.linear.x = -0.4
                cmd.angular.z = 0.5 * angle_diff
                print("🚗 Driving forward...")
        else:
            self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}")
            self.get_logger().info(f"   Final Pose: x={pos.x:.2f}, y={pos.y:.2f}, yaw={self.yaw:.2f}")
            self.publisher_.publish(Twist())  # full stop
            self.current_index += 1
            self.target_angle = None
            self.rotating_to_next = True  # activate turn-to-next mode
            return

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()

# == v2
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion
# import math

# def read_waypoints(file_path):
#     waypoints = []
#     with open(file_path, 'r') as f:
#         for line in f:
#             line = line.strip()
#             if not line or line.startswith('#'):
#                 continue
#             parts = list(map(float, line.split()))
#             if len(parts) == 2:
#                 waypoints.append((parts[0], parts[1]))
#     print("✅ Waypoints loaded:", waypoints)
#     return waypoints

# class Navigator(Node):
#     def __init__(self):
#         super().__init__('navigator')
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
#         self.timer = self.create_timer(5, self.control_loop)
#         self.pose = None
#         self.yaw = 0.0
#         self.waypoints = read_waypoints('./waypoints1.txt')
#         self.current_index = 0
#         self.target_angle = None
#         self.rotating_to_next = True
#         self.rotation_direction = None  # "left" or "right"
#         self.initial_position_printed = False

#     def odom_callback(self, msg):
#         self.pose = msg.pose.pose
#         orientation_q = self.pose.orientation
#         _, _, self.yaw = euler_from_quaternion([
#             orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
#         ])
#         if not self.initial_position_printed:
#             print(f"📍 Initial Pose: x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f}, yaw={self.yaw:.2f}")
#             self.initial_position_printed = True

#     def control_loop(self):
#         if self.pose is None or self.current_index >= len(self.waypoints):
#             return

#         pos = self.pose.position
#         x_target, y_target = self.waypoints[self.current_index]

#         dx = x_target - pos.x
#         dy = y_target - pos.y
#         distance = math.hypot(dx, dy)

#         if self.target_angle is None:
#             self.target_angle = math.atan2(dy, dx)
#             print(f"\n➡️ Waypoint {self.current_index + 1}")
#             print(f"Target position: ({x_target:.2f}, {y_target:.2f})")
#             print(f"Current robot position: ({pos.x:.2f}, {pos.y:.2f})")
#             print(f"dx = {dx:.2f}, dy = {dy:.2f}, target_angle = {self.target_angle:.2f}")

#         diff = self.target_angle - self.yaw
#         angle_diff = math.atan2(math.sin(diff), math.cos(diff))

#         print(f"Current yaw: {self.yaw:.2f}, Angle diff: {angle_diff:.2f}, Distance: {distance:.2f}")

#         cmd = Twist()

#         # ✅ Handle post-waypoint reorientation
#         if self.rotating_to_next:
#             if self.rotation_direction is None:
#                 self.rotation_direction = "left" if angle_diff > 0 else "right"
#                 print(f"🔐 Locking rotation direction: {self.rotation_direction}")

#             if abs(angle_diff) > 0.05:
#                 cmd.linear.x = 0.0
#                 cmd.angular.z = 1.3 * abs(angle_diff) if self.rotation_direction == "left" else -1.3 * abs(angle_diff)
#                 print("🔄 Rotating to face next waypoint...")
#                 self.publisher_.publish(cmd)
#                 return
#             else:
#                 print("✅ Rotation complete. Resuming navigation.")
#                 self.rotating_to_next = False
#                 self.rotation_direction = None

#         if abs(abs(angle_diff) - math.pi) < 0.05:
#             print(" Near-pi wraparound suppressed")
#             angle_diff = 0.0


#         print(f"Current yaw: {self.yaw:.2f}, Angle diff wrap around:  {angle_diff:.2f}, Distance: {distance:.2f}")
#         # 🚗 Normal navigation logic
#         if distance > 0.3:
#             cmd.linear.x = -0.4
#             cmd.angular.z = 0.5 * angle_diff
#             print("🚗 Driving forward...")
#         else:
#             self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}")
#             self.get_logger().info(f"   Final Pose: x={pos.x:.2f}, y={pos.y:.2f}, yaw={self.yaw:.2f}")
#             self.publisher_.publish(Twist())
#             self.current_index += 1
#             self.target_angle = None
#             self.rotating_to_next = True
#             self.rotation_direction = None
#             return

#         self.publisher_.publish(cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     node = Navigator()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# main()
