#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

class MazeVisualizer:
    def __init__(self):
        rospy.init_node('maze_visualizer')

        self.vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.loginfo("MazeVisualizer initialized.")

    def visualize(self):
        sensed_maze = rospy.get_param("~sensed_maze", [])
        for y, row in enumerate(sensed_maze):
            for x, cell in enumerate(row):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.5
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 1.0 if cell == 1 else 0.0
                marker.color.g = 1.0 if cell == 0 else 0.0
                self.vis_pub.publish(marker)

if __name__ == "__main__":
    try:
        visualizer = MazeVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MazeVisualizer node terminated.")
