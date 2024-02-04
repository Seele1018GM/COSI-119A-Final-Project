# Turtlebot3 Maze Solver & Mapper

## Project Description
    
In ROS1 noetic, entry-level maze solver and maze mapper nodes have been developed for TurtleBot3 in Micromouse maze environments.

The project utilizes the following ROS packages: geometry_msgs, roscpp, sensor_msgs, std_msgs, as well as OpenCV.

Preparing the maze environment in Gazebo:

```bash
$ mkdir -p ~/tb3_maze_solver/src
$ cd ~/tb3_maze_solver/src
$ git clone https://github.com/fbasatemur/turtlebot3_maze_solver.git
$ cd ..
$ catkin_make
$ source ~/.bashrc
$ echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
$ source ~/.bashrc
$ echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/tb3_maze_solver/src/" >> ~/.bashrc
$ source ~/.bashrc

$ roslaunch micromouse_maze micromouse_maze1.launch
$ roslaunch micromouse_maze micromouse_maze2.launch
```

To run the solver & mapper nodes:

```bash
rosrun solve_maze my_solver
rosrun solve_maze my_mapper
```

To see the mapper result in detail, you can open the Robot Steering Console by running:

```bash
rosrun rqt_gui rqt_gui
```
Then navigate to Plugins -> Robot Tools -> Robot Steering.

## Solver

### Avoiding Obstacles
The robot performs obstacle scanning to ensure the absence of LiDAR data within a specified area (a x b) in front of it. This scanning relies on angular calculations expressed as follows:

![odom_png](https://github.com/fbasatemur/turtlebot3_maze_solver/blob/main/doc/odom.png?ref_type=heads)


In Fig-1, calculations for the minimum and maximum angles required for scanning an area of size axb in front of the TurtleBot are depicted. The dimensions of the TurtleBot (turtlebot_l and turtlebot_w) are used in Fig-2 to compute the angle Q_rad_min.

After representing the circular area, LiDAR data is transformed into Cartesian coordinates, and obstacle checking is performed as follows:

```c
if (x <= a_2 and (y <= (turtlebot_l_2 + b) and y >= turtlebot_l_2)) 
```

If the above condition is met, the robot determines that there is an obstacle in front of it and turns 10 degrees on the z-axis.

### Note

The TurtleBot is defaultly sized at 0.281 x 0.306. To disregard LiDAR data outside the area, lidar_max_range_th is set to 1.5. Additionally, a PID controller has been implemented for centering the robot's path, but the coefficients are left as comments in the code since they are not appropriately adjusted.

Due to time constraints, features such as the use of A* and BDF-like shortest path algorithms and offline map generation between 2 points are not implemented to enable the robot to solve the micromouse_maze.

## Mapper

The my_mapper node is designed to map the environment for TurtleBot's navigation within a labyrinth. This node subscribes to the /odom and /scan messages, utilizing the data obtained from these messages to generate the environment map.

### Coordinat Transformations
The range data obtained from the /scan message is transformed into the horizontal and vertical axes based on the orientation of the TurtleBot in the z-axis. This transformation is carried out according to the following control structure:

```c
if(turtle_ori_z <= 0.35 or turtle_ori_z >= 0.85){
    // tb3 moves only along the vertical axis
    cart_x = range * cos(angle);
    cart_y = range * sin(angle);
}else{
    // tb3 moves only along the horizontal axis
    cart_x = range * sin(angle);
    cart_y = range * cos(angle);
}
```

Subsequently, the calculated Cartesian coordinates are first transformed to the base_footprint frame and then to the odom frame. Finally, the obtained odom data is combined with the robot's pose information and transferred to the environment.

```cpp
obstacle_x = static_cast<int>(turtle_pos_x + odom_x[i] / scan_resolution);
obstacle_y = static_cast<int>(turtle_pos_y + odom_y[i] / scan_resolution);
```

This node maps the obstacles within the labyrinth based on the environment map created according to the micromouse_maze puzzle. The map contains the positions of obstacles perceived by the TurtleBot.

### micromouse_maze1 Obstacle Map
![maze1_png](https://github.com/fbasatemur/turtlebot3_maze_solver/blob/main/doc/maze_1_mapper.png?ref_type=heads)

