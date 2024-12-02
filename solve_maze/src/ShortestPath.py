#!/usr/bin/env python3

import rospy
import heapq
from collections import defaultdict

class EnhancedPathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')
        
        # Initialize state
        self.current_position = (0, 0)
        self.goal = (7, 7)
        self.maze_size = 16
        
        # Create a subscriber for flood fill data
        # We'll need to create a custom message type for this
        self.flood_sub = rospy.Subscriber('/flood_fill_data', FloodFillData, self.flood_data_callback)
        self.cmd_pub = rospy.Publisher('/move_commands', String, queue_size=10)

        # Store flood fill information
        self.distance_costs = [[float('inf')] * self.maze_size for _ in range(self.maze_size)]
        self.sensed_maze = [[-1] * self.maze_size for _ in range(self.maze_size)]

    def flood_data_callback(self, msg):
        """Receive updates from flood fill node"""
        # Update our local copy of the flood fill data
        self.distance_costs = msg.distance_costs
        self.sensed_maze = msg.sensed_maze
        self.current_position = msg.current_position
        
        # Plan path based on updated information
        self.plan_path()

    def plan_path(self):
        """Plan path using flood fill information"""
        # Use flood fill costs as base costs
        current = self.current_position
        goal = self.goal
        
        # Find path that minimizes combination of:
        # 1. Flood fill distance costs
        # 2. Path length
        # 3. Exploration potential
        path = self.find_optimal_path(current, goal)
        
        if path:
            self.execute_next_move(path[0])