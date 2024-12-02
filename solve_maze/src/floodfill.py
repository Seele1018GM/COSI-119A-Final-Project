#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray, Int8MultiArray
from geometry_msgs.msg import Point
from collections import deque
import math
import numpy as np

class DynamicFloodFillSolver:
    def __init__(self):
        """Initialize the flood fill solver with maze parameters and ROS communications"""
        rospy.init_node('dynamic_floodfill_solver', anonymous=True)

        # Robot State Configuration
        self.current_position = (0, 0)  # Starting position in grid coordinates
        self.goal = (7, 7)              # Center of 16x16 maze
        self.grid_size = 1.0            # Each grid cell is 1x1 meter
        self.maze_size = 16             # 16x16 grid
        
        # Maze Data Structures
        # Using -1 for unknown, 0 for open space, 1 for wall, 2 for visited
        self.sensed_maze = [[-1 for _ in range(self.maze_size)] for _ in range(self.maze_size)]
        self.distance_costs = [[float('inf') for _ in range(self.maze_size)] for _ in range(self.maze_size)]
        self.wall_certainty = [[0.0 for _ in range(self.maze_size)] for _ in range(self.maze_size)]
        
        # LIDAR Configuration Parameters
        self.scan_angle = 20            # Degrees to check on either side of cardinal directions
        self.min_wall_range = 0.15      # Minimum distance for wall detection
        self.max_wall_range = 0.45      # Maximum distance for wall detection
        self.wall_confidence_threshold = 0.7  # Threshold for confirming wall presence

        # Set up ROS communications
        self.setup_ros_communications()
        
        rospy.loginfo("DynamicFloodFillSolver initialized and ready")

    def setup_ros_communications(self):
        """Set up all ROS publishers and subscribers"""
        # Subscribers for sensor data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publishers for maze data and movement commands
        self.command_pub = rospy.Publisher('/move_commands', String, queue_size=10)
        self.flood_data_pub = rospy.Publisher('/flood_fill_data', Float32MultiArray, queue_size=10)
        self.maze_pub = rospy.Publisher('/sensed_maze', Int8MultiArray, queue_size=10)

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates with offset handling"""
        # Add offset to handle negative coordinates
        offset_x = x + (self.maze_size * self.grid_size / 2)
        offset_y = y + (self.maze_size * self.grid_size / 2)
        
        # Round to nearest grid cell to handle slight imprecisions
        grid_x = int(round(offset_x / self.grid_size))
        grid_y = int(round(offset_y / self.grid_size))
        
        # Ensure coordinates are within maze bounds
        grid_x = max(0, min(grid_x, self.maze_size - 1))
        grid_y = max(0, min(grid_y, self.maze_size - 1))
        
        return (grid_x, grid_y)

    def process_wall_detection(self, readings, direction):
        """Process LIDAR readings for a specific direction to detect walls"""
        valid_readings = [r for r in readings if self.min_wall_range <= r <= self.max_wall_range]
        
        if not readings:  # Handle empty readings
            return False, 0.0
            
        wall_confidence = len(valid_readings) / len(readings)
        wall_detected = wall_confidence > self.wall_confidence_threshold
        
        return wall_detected, wall_confidence

    def update_maze_from_lidar(self, msg):
        """Update maze understanding using LIDAR data with confidence tracking"""
        # Process readings for each cardinal direction
        directions = [0, 90, 180, 270]  # Degrees (Front, Left, Back, Right)
        walls_detected = []
        
        for direction in directions:
            # Calculate indices for the reading range
            start_idx = int((direction - self.scan_angle) * len(msg.ranges) / 360)
            end_idx = int((direction + self.scan_angle) * len(msg.ranges) / 360)
            
            # Handle wrap-around for complete 360° scan
            if end_idx > len(msg.ranges):
                readings = msg.ranges[start_idx:] + msg.ranges[:end_idx - len(msg.ranges)]
            else:
                readings = msg.ranges[start_idx:end_idx]
            
            wall_detected, confidence = self.process_wall_detection(readings, direction)
            walls_detected.append((wall_detected, confidence))

        # Update maze with detected walls
        x, y = self.current_position
        self.update_cell_states(x, y, walls_detected)

    def update_cell_states(self, x, y, walls_detected):
        """Update maze cells based on wall detection results"""
        # Define relative positions for adjacent cells
        adjacents = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # N, W, S, E
        
        for (wall_detected, confidence), (dx, dy) in zip(walls_detected, adjacents):
            new_x, new_y = x + dx, y + dy
            
            # Skip if outside maze bounds
            if not (0 <= new_x < self.maze_size and 0 <= new_y < self.maze_size):
                continue
                
            # Update wall certainty
            old_certainty = self.wall_certainty[new_y][new_x]
            new_certainty = old_certainty + confidence * (1 - old_certainty)
            self.wall_certainty[new_y][new_x] = new_certainty
            
            # Update maze state if confidence is high enough
            if new_certainty > 0.8:
                self.sensed_maze[new_y][new_x] = 1 if wall_detected else 0

    def flood_fill(self):
        """Perform flood fill algorithm to calculate optimal paths to goal"""
        # Reset distance costs
        self.distance_costs = [[float('inf')] * self.maze_size for _ in range(self.maze_size)]
        
        # Initialize queue with goal position
        goal_x, goal_y = self.goal
        queue = deque([(goal_x, goal_y)])
        self.distance_costs[goal_y][goal_x] = 0

        while queue:
            x, y = queue.popleft()
            current_cost = self.distance_costs[y][x]

            # Process all valid neighbors
            for nx, ny in self.get_neighbors(x, y):
                # Calculate new cost including penalties
                new_cost = current_cost + 1
                
                # Update cost if new path is better
                if new_cost < self.distance_costs[ny][nx]:
                    self.distance_costs[ny][nx] = new_cost
                    queue.append((nx, ny))

    def get_neighbors(self, x, y):
        """Get valid neighboring cells that can be moved to"""
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:  # Four directions
            nx, ny = x + dx, y + dy
            
            # Check bounds and wall status
            if (0 <= nx < self.maze_size and 
                0 <= ny < self.maze_size and 
                self.sensed_maze[ny][nx] != 1):  # Not a wall
                neighbors.append((nx, ny))
        
        return neighbors

    def navigate(self):
        """Determine next move based on flood fill results"""
        x, y = self.current_position
        neighbors = self.get_neighbors(x, y)
        
        # Filter out visited cells unless no other option
        valid_neighbors = [n for n in neighbors if self.sensed_maze[n[1]][n[0]] != 2]
        if not valid_neighbors:
            valid_neighbors = neighbors  # Fall back to visited cells if necessary
            
        if not valid_neighbors:
            rospy.loginfo("No valid neighbors to navigate to. Stopping.")
            return

        # Choose next cell based on minimum distance cost
        next_cell = min(valid_neighbors, 
                       key=lambda cell: self.distance_costs[cell[1]][cell[0]])
        
        # Determine movement direction
        self.publish_movement_command(x, y, next_cell)
        
        # Mark current cell as visited
        self.sensed_maze[y][x] = 2

    def publish_movement_command(self, current_x, current_y, next_cell):
        """Publish movement command based on direction to next cell"""
        dx = next_cell[0] - current_x
        dy = next_cell[1] - current_y

        if dx == 1:
            rospy.loginfo("Move Right")
            self.command_pub.publish("right")
        elif dx == -1:
            rospy.loginfo("Move Left")
            self.command_pub.publish("left")
        elif dy == -1:
            rospy.loginfo("Move Up")
            self.command_pub.publish("up")
        elif dy == 1:
            rospy.loginfo("Move Down")
            self.command_pub.publish("down")

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_position = self.world_to_grid(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def scan_callback(self, msg):
        """Handle new LIDAR scan data"""
        rospy.loginfo("Processing new LIDAR scan...")
        self.update_maze_from_lidar(msg)
        self.flood_fill()
        self.navigate()
        self.publish_maze_data()

    def publish_maze_data(self):
        """Publish current maze state and flood fill data"""
        # Publish flood fill distances
        flood_msg = Float32MultiArray()
        flood_msg.data = [item for sublist in self.distance_costs for item in sublist]
        self.flood_data_pub.publish(flood_msg)

        # Publish maze state
        maze_msg = Int8MultiArray()
        maze_msg.data = [item for sublist in self.sensed_maze for item in sublist]
        self.maze_pub.publish(maze_msg)

    def spin(self):
        """Start the node's main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        solver = DynamicFloodFillSolver()
        solver.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("DynamicFloodFillSolver node terminated.")