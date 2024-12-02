#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import tf.transformations

class MovementControl:
    def __init__(self):
        rospy.init_node("movement_control", anonymous=True)

        # Control parameters for smooth movement
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.position_tolerance = 0.02  # 2cm position tolerance
        self.angle_tolerance = math.radians(2)  # 2 degree angle tolerance

        # Current state tracking
        self.current_pose = Pose()
        self.current_orientation = 0.0  # In radians
        self.laser_ranges = []
        self.is_moving = False

        # ROS communication setup
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.command_sub = rospy.Subscriber('/move_commands', String, self.execute_command)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.loginfo("MovementControl node initialized.")

    def odom_callback(self, msg):
        """Update current position and orientation from odometry data."""
        self.current_pose = msg.pose.pose
        # Extract yaw angle from quaternion
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_orientation = euler[2]  # yaw angle

    def laser_callback(self, msg):
        """Store laser scan data for obstacle detection."""
        self.laser_ranges = msg.ranges

    def execute_command(self, msg):
        """Handle movement commands with improved error checking."""
        if self.is_moving:
            rospy.logwarn("Already executing a movement command. Ignoring new command.")
            return

        self.is_moving = True
        command = msg.data
        
        try:
            if command == "forward" or command == "up":
                self.move_forward(1.0)
            elif command == "left":
                self.rotate_to_angle(math.pi/2)  # 90 degrees left
            elif command == "right":
                self.rotate_to_angle(-math.pi/2)  # 90 degrees right
            elif command == "down":
                self.rotate_to_angle(math.pi)  # 180 degrees
        except Exception as e:
            rospy.logerr(f"Error executing movement command: {str(e)}")
        finally:
            self.is_moving = False

    def calculate_distance(self, start_pose, current_pose):
        """Calculate Euclidean distance between two poses."""
        dx = current_pose.position.x - start_pose.position.x
        dy = current_pose.position.y - start_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_forward(self, target_distance):
        """Execute forward movement with closed-loop control."""
        start_pose = self.current_pose
        rate = rospy.Rate(20)  # 20Hz control loop
        
        while not rospy.is_shutdown():
            # Calculate remaining distance
            current_distance = self.calculate_distance(start_pose, self.current_pose)
            remaining = target_distance - current_distance

            # Check if we've reached the target
            if abs(remaining) < self.position_tolerance:
                self.stop()
                break

            # Proportional control for smooth acceleration and deceleration
            speed = min(self.linear_speed, max(0.05, abs(remaining) * 0.5))
            
            # Create and publish movement command
            twist = Twist()
            twist.linear.x = speed if remaining > 0 else -speed
            self.cmd_pub.publish(twist)
            rate.sleep()

    def rotate_to_angle(self, target_angle_change):
        """Execute rotation with closed-loop control."""
        target_angle = self.normalize_angle(self.current_orientation + target_angle_change)
        rate = rospy.Rate(20)  # 20Hz control loop

        while not rospy.is_shutdown():
            # Calculate remaining angle
            angle_diff = self.normalize_angle(target_angle - self.current_orientation)
            
            # Check if we've reached the target angle
            if abs(angle_diff) < self.angle_tolerance:
                self.stop()
                break

            # Proportional control for smooth rotation
            speed = min(self.angular_speed, max(0.1, abs(angle_diff) * 0.5))
            
            # Create and publish movement command
            twist = Twist()
            twist.angular.z = speed if angle_diff > 0 else -speed
            self.cmd_pub.publish(twist)
            rate.sleep()

    def stop(self):
        """Stop all robot movement."""
        twist = Twist()
        # Publish stop command multiple times to ensure it's received
        for _ in range(3):
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        control = MovementControl()
        control.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MovementControl node terminated.")