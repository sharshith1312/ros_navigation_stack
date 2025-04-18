#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class WaypointFollower(Node):
    def __init__(self, waypoint_file):
        super().__init__('waypoint_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoints = self.load_waypoints(waypoint_file)
        self.timer = self.create_timer(1.0, self.send_next_command)
        self.index = 0

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue  # Skip empty lines and comments
                parts = list(map(float, line.split()))
                if len(parts) == 3:
                    waypoints.append(parts)
        return waypoints


    def send_next_command(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('All waypoints sent. Stopping robot.')
            self.stop_robot()
            rclpy.shutdown()
            return

        linear_x, angular_z, duration = self.waypoints[self.index]
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.get_logger().info(f'Sending: linear_x={linear_x}, angular_z={angular_z}, duration={duration}')
        self.publisher.publish(msg)
        self.index += 1

        time.sleep(duration)  # blocking sleep (simple for demo)

    def stop_robot(self):
        stop_msg = Twist()
        self.publisher.publish(stop_msg)



def main():
    rclpy.init()
    waypoint_file = 'waypoints.txt'
    node = WaypointFollower(waypoint_file)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
