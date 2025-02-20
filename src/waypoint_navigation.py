#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')
        
        # Declare ROS 2 parameters for waypoints, PID gains, and obstacle avoidance threshold
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', 3.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('obstacle_distance', 0.5)  # Stop if an obstacle is closer than 0.5m
        
        # Load parameter values
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        # PID control variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_waypoint = 0
        self.obstacle_detected = False  # Flag for obstacle detection

        # Create subscribers and publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)  # LiDAR
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Waypoint Navigator with Obstacle Avoidance Started')

    def scan_callback(self, msg):
        # Check if there is an obstacle in front of the robot
        min_distance = min(msg.ranges) if msg.ranges else float('inf')
        if min_distance < self.obstacle_distance:
            self.obstacle_detected = True
            self.get_logger().warn(f'Obstacle detected! Distance: {min_distance:.2f}m')
        else:
            self.obstacle_detected = False

    def odom_callback(self, msg):
        # If all waypoints have been reached, stop the robot.
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping robot.')
            self.stop_robot()
            return

        # Get current position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        target_x, target_y = self.waypoints[self.current_waypoint]

        # Compute distance to target waypoint
        error = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        # Stop movement if an obstacle is detected
        if self.obstacle_detected:
            self.stop_robot()
            return

        # PID control for linear speed
        self.integral += error
        derivative = error - self.prev_error
        speed = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Compute desired heading angle
        angle_to_target = math.atan2(target_y - y, target_x - x)
        current_angle = msg.pose.pose.orientation.z

        twist = Twist()
        twist.linear.x = min(speed, 0.5)  # Limit max linear speed
        twist.angular.z = angle_to_target - current_angle

        self.cmd_vel_pub.publish(twist)

        # Check if close enough to switch to the next waypoint
        if error < 0.2:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}. Moving to next.')
            self.current_waypoint += 1

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0  # Turn to avoid obstacle
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
