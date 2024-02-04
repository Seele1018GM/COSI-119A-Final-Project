#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>


class Mapper{
    double pose_x, pose_y, ori_z;
    bool is_odom_msg_generated;
    sensor_msgs::LaserScan scan_msg;

    int width, height, win_center_x, win_center_y;
    double scan_resolution, pos_resolution;

    ros::Subscriber scan_sub, odom_sub;
    ros::NodeHandle nh;
    float range_th{.9F};
    cv::Mat map;
public:

    Mapper(): nh("~") {
        width = 800;
        height = 800;
        win_center_x = width / 2;
        win_center_y = height / 2;
        pos_resolution = std::min(width, height) / 18;  // Scaling factor for positioning pose information
        scan_resolution = 0.04;                         // Conversion rate from meters to pixels (1 meter = 25 pixels)
        map = cv::Mat{height, width, CV_8UC3, cv::Scalar(255, 255, 255)};
        cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);

        is_odom_msg_generated = false;

        scan_sub = nh.subscribe("/scan", 1, &Mapper::scanCallback, this);
        odom_sub = nh.subscribe("/odom", 1, &Mapper::odomCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg_p) {

        scan_msg = *scan_msg_p;
        float angle_min = scan_msg.angle_min;
        float angle_increment = scan_msg.angle_increment;
        float angle{angle_min};
        std::vector<float> cartesian_x, cartesian_y;
        float cart_x, cart_y, turtle_ori_z;

        // If an odom message is received,
        if (is_odom_msg_generated)
        {
            turtle_ori_z = std::abs(ori_z);

            // Convert the measurements from the /scan topic to Cartesian coordinates.
            for (auto range: scan_msg.ranges) {

                if (range < range_th){
                    
                    if(turtle_ori_z <= 0.35 or turtle_ori_z >= 0.85){
                        // tb3 moves only along the vertical axis
                        cart_x = range * cos(angle);
                        cart_y = range * sin(angle);
                    }else{
                        // tb3 moves only along the horizontal axis
                        cart_x = range * sin(angle);
                        cart_y = range * cos(angle);
                    }
                    cartesian_x.push_back(cart_x);
                    cartesian_y.push_back(cart_y);
                }
                angle += angle_increment;
            }

            // Conversion from the base_scan frame to the base_footprint frame
            std::vector<float> base_fp_x, base_fp_y, base_fp_z;
            for (size_t i = 0; i < cartesian_x.size(); ++i) {
                base_fp_x.push_back(cartesian_x[i] - 0.064);
                base_fp_y.push_back(cartesian_y[i]);
                base_fp_z.push_back(0.132);
            }

            // Conversion from the base_footprint frame to the odom frame
            tf2::Matrix3x3 rotation_matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, 0);
            rotation_matrix.setRotation(quaternion);
            tf2::Vector3 translation_vector(-0.064, 0.0, 0.132);

            std::vector<float> odom_x, odom_y, odom_z;
            for (size_t i = 0; i < base_fp_x.size(); ++i) {
                tf2::Vector3 point(base_fp_x[i], base_fp_y[i], base_fp_z[i]);
                point = rotation_matrix * point + translation_vector;
                odom_x.push_back(point.getX());
                odom_y.push_back(point.getY());
                odom_z.push_back(point.getZ());
            }

            angle = angle_min;
            float turtle_pos_x, turtle_pos_y;
            int obstacle_x, obstacle_y;

            turtle_pos_x = pose_x * pos_resolution + win_center_x;
            turtle_pos_y = pose_y * pos_resolution + win_center_y;

            for (int i = 0; i < odom_x.size(); i++) {
                // Calculate the coordinates of the obstacle
                obstacle_x = static_cast<int>(turtle_pos_x + odom_x[i] / scan_resolution);
                obstacle_y = static_cast<int>(turtle_pos_y + odom_y[i] / scan_resolution);

                // Draw the obstacles within the boundaries of the map
                if (obstacle_x >= 0 && obstacle_x < width && obstacle_y >= 0 && obstacle_y < height)
                    map.at<cv::Vec3b>(obstacle_y, obstacle_x) = cv::Vec3b(0, 0, 0);
                
                angle += angle_increment;
            }
        }

        cv::imshow("Map", map);
        cv::waitKey(1);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
        pose_x = odom_msg->pose.pose.position.x;
        pose_y = odom_msg->pose.pose.position.y;
        ori_z = odom_msg->pose.pose.orientation.z;
        is_odom_msg_generated = true;
    }

    ~Mapper(){
        cv::destroyWindow("Map");
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper_node");

    Mapper mapper;

    ros::spin();
    return 0;
}
