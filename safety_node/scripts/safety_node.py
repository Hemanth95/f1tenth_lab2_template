#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.ttc_threshold = 1.5
        self.ttc_min = float('inf')
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.pub_brake = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.pub_brake_bool = self.create_publisher(Bool, '/brake_bool', 10)
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Reset the minimum TTC for each scan
        self.ttc_min = float('inf')
        
        # Calculate TTC for each laser scan range
        for i, range in enumerate(scan_msg.ranges):
            if range <= 0.0 or np.isinf(range) or np.isnan(range): 
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            relative_speed = self.speed * np.cos(angle)

            # Calculate TTC
            ttc = range / abs(relative_speed)
            self.ttc_min = min(self.ttc_min, ttc)
            # Reverse motion check
        if self.speed < 0 and self.ttc_min < 1.5:
            self.get_logger().info('Braking for reverse motion')
            self.publish_brake()
            return
        
        # Forward motion check
        elif self.speed > 0 and self.ttc_min < 1.8:
            self.get_logger().info('Braking for forward motion')
            self.publish_brake()
            return

    def publish_brake(self):
        drive_msg = AckermannDriveStamped()
        drive_bool = Bool()
        drive_bool.data = True
        drive_msg.drive.speed = 0.0
        self.pub_brake_bool.publish(drive_bool)
        self.pub_brake.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()