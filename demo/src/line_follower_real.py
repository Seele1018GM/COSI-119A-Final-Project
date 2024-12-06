#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point

class LineFollowerReal:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Twist for velocity commands
        self.twist = Twist()
        self.current_dist = 0.0
        self.cur_yaw = 0.0

    def my_odom_cb(self, msg):
        """Callback for odometry updates."""
        self.current_dist = msg.x
        self.cur_yaw = msg.y

    def image_cb(self, msg):
        """Callback to process camera image."""
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Convert to HSV for line detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0, 0, 0])
        upper_yellow = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find largest contour
            line_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(line_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                err = cx - image.shape[1] // 2

                # Draw line and centroid
                cv2.drawContours(image, [line_contour], -1, (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

                # Publish velocity commands
                self.twist.linear.x = 0.2
                self.twist.angular.z = -err * 0.003
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.stop_robot()
        else:
            rospy.logwarn("No line detected. Stopping robot.")
            self.stop_robot()

        # Display the image
        cv2.imshow("Line Follower", image)
        cv2.waitKey(3)

    def stop_robot(self):
        """Stop the robot."""
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        """Run the robot behavior."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
