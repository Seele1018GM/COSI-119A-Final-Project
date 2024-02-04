#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

class Solver {
public:
    Solver() : nh("~") {
        scan_sub = nh.subscribe("/scan", 1, &Solver::scanCallback, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // Dimensions of the obstacle (width and length)
        a = 0.3; // width
        b = 0.2; // height

        // Default TurtleBot dimensions
        double turtlebot_l = 0.281, turtlebot_w = 0.306; 

        a_2 = a / 2, b_2 = b / 2;
        turtlebot_l_2 = turtlebot_l / 2, turtlebot_w_2 = turtlebot_w / 2;
        
        // Calculation of angle intervals for area scanning
        Q_rad_min = M_PI / 2 - std::atan2(turtlebot_l_2, a_2);
        Q_rad_max = 2 * M_PI - Q_rad_min;

        rotate_vel = 0.1745;      // 10 degree
        straight_vel = 0.2;

        // Acceptable maximum LiDAR range
        lidar_max_range_th = 1.5f;

        // PID controller parameters
        kp = 0.5;  
        ki = 0.01; 
        kd = 0.1;  
        prev_error = 0;
        integral = 0;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg_p) {

        scan_msg = *scan_msg_p;

        // If there is an obstacle ahead, rotate around itself
        if (not checkObstacle()) {
            turnRobot();
        } else {
            moveStraight();
        }
    }

    float getPIDError(){
        
        // LiDAR data is used to control the distance to obstacles
        float left_distance = scan_msg.ranges[89];
        float right_distance = scan_msg.ranges[269];
        float pid_output{0};

        float diff_dist = left_distance - right_distance;
        if(left_distance < lidar_max_range_th and right_distance < lidar_max_range_th){

            float error = diff_dist;
            float derivative = error - prev_error;
            
            integral += error;
            prev_error = error;

            pid_output = (kp * error) + (ki * integral) + (kd * derivative);
        }

        return pid_output;
    }

    bool checkObstacle() {
        // Returns false if there is an obstacle; else true
        double angle_increment = scan_msg.angle_increment;
        double angle = scan_msg.angle_min;
        double x, y;

        for (auto range: scan_msg.ranges) {
            if (angle >= Q_rad_max or angle <= Q_rad_min) {
                y = range * cos(angle);
                x = range * sin(angle);

                if (x <= a_2 and (y <= (turtlebot_l_2 + b) and y >= turtlebot_l_2)) {
                    // the area is not empty
                    return false;
                }
            }
            angle += angle_increment;
        }

        // If there is no obstacle in the axb area, the space is empty
        return true;
    }


    void turnRobot() {
        cmd_vel_msg.angular.z = rotate_vel; 
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::Duration(.2).sleep();
    }

    void moveStraight() {
        /* 
            The PID controller is written to center the TurtleBot's path, 
            but it is left as a comment since it is not fully adjusted. 
        */
        // cmd_vel_msg.angular.z = getPIDError();  // Angular velocity (in the z-axis)
        cmd_vel_msg.linear.x = straight_vel;
        cmd_vel_msg.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_msg;
    double Q_rad_min, Q_rad_max;
    double turtlebot_l_2, turtlebot_w_2;
    float a, b, a_2, b_2;                       
    float rotate_vel, straight_vel;            
    float kp, ki, kd, prev_error, integral;     
    float lidar_max_range_th;
    sensor_msgs::LaserScan scan_msg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "solver_node");
    Solver solver;
    ros::spin();
    return 0;
}
