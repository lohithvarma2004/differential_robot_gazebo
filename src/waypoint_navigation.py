#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')
        
        # Declare ROS 2 parameters for waypoints, PID gains, and obstacle avoidance
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', 3.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('obstacle_distance', 0.8)  # Increased avoidance tolerance
        
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
        self.avoiding_obstacle = False  # State flag for obstacle avoidance
        self.avoidance_bias = 0.0  # Helps prevent collisions after avoidance

        # Create subscribers and publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)  # LiDAR
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Waypoint Navigator with Stronger Obstacle Avoidance Started')

    def scan_callback(self, msg):
        """Processes LiDAR data to detect obstacles and adjust navigation."""
        if not msg.ranges or len(msg.ranges) < 3:
            return  # Ignore empty LiDAR readings

        num_ranges = len(msg.ranges)
        range_max = msg.range_max  # Maximum LiDAR range
        filtered_ranges = [r if r > 0 else range_max for r in msg.ranges]  # Replace zero values (faulty readings)

        # Divide LiDAR scan into three regions and use median filtering to avoid noise
        right = sorted(filtered_ranges[0:int(num_ranges * 0.3)])[int(num_ranges * 0.15)]
        center = sorted(filtered_ranges[int(num_ranges * 0.3):int(num_ranges * 0.7)])[int(num_ranges * 0.2)]
        left = sorted(filtered_ranges[int(num_ranges * 0.7):])[int(num_ranges * 0.15)]

        self.get_logger().info(f'LiDAR: Left={left:.2f}m, Center={center:.2f}m, Right={right:.2f}m')

        # Increased detection buffer
        safety_buffer = 0.5  
        dynamic_threshold = max(0.5, min(self.obstacle_distance + safety_buffer, center * 0.8))  

        # Obstacle avoidance logic
        if center < dynamic_threshold:
            self.avoid_obstacle("right" if right > left else "left", center)
        elif left < self.obstacle_distance + safety_buffer:
            self.avoid_obstacle("right", left)
        elif right < self.obstacle_distance + safety_buffer:
            self.avoid_obstacle("left", right)
        else:
            self.avoiding_obstacle = False  # No obstacles detected
            self.avoidance_bias = 0.0  # Reset avoidance bias

    def avoid_obstacle(self, direction, distance):
        """Adjusts motion to avoid an obstacle smoothly and aggressively if needed."""
        self.avoiding_obstacle = True
        twist = Twist()

        # Reduce speed dynamically based on distance to obstacle
        twist.linear.x = max(0.03, 0.12 * (distance / self.obstacle_distance))

        # Stronger turn response based on proximity
        turn_speed = max(0.4, 1.5 * (1.0 - distance / (self.obstacle_distance + 0.2)))
        turn_speed = min(turn_speed, 1.2)  # Cap max turn speed
        twist.angular.z = turn_speed if direction == "left" else -turn_speed

        # Introduce avoidance bias to prevent brushing obstacles
        self.avoidance_bias = 0.3 if direction == "left" else -0.3

        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn(f'Obstacle detected! Turning {direction} with turn speed {twist.angular.z:.2f}.')

    def odom_callback(self, msg):
        """Moves towards waypoints, considering obstacle avoidance."""
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping robot.')
            self.stop_robot()
            return

        if self.avoiding_obstacle:
            return  # Let obstacle avoidance take control

        # Get current position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        target_x, target_y = self.waypoints[self.current_waypoint]

        # Compute distance to target waypoint
        error = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        # PID control for linear speed
        self.integral += error
        derivative = error - self.prev_error
        speed = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Compute desired heading angle
        angle_to_target = math.atan2(target_y - y, target_x - x)
        current_angle = msg.pose.pose.orientation.z

        twist = Twist()
        twist.linear.x = min(speed, 0.3)  # Reduce speed for safety
        twist.angular.z = angle_to_target - current_angle + self.avoidance_bias  # Apply avoidance bias

        self.cmd_vel_pub.publish(twist)

        # Check if close enough to switch to the next waypoint
        if error < 0.3:  # Increased tolerance for smoother transitions
            self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}. Moving to next.')
            self.current_waypoint += 1

    def stop_robot(self):
        """Stops the robot by publishing zero velocity."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
