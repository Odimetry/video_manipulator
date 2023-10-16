#ifndef COLOR_MANIPULATOR_HPP_
#define COLOR_MANIPULATOR_HPP_
// hpp define start:

#include "ros/ros.h"
#include <string>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
// Message header file

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
// Manipulator header file

#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>
// OpenCV header file

using namespace std;
using namespace cv;

namespace final { // about "final" namespace
class Node // define "Node" class
{
    public:
        Node(int argc, char** argv); // Object Contructor
        virtual ~Node(); // Object Distructor
        bool init(); // ROS Node
        enum LogLevel {
            Debug,
            Info,
            Warn,
            Error,
            Fatal
        };
        void move(); // Turtlebot's moving function

        // Defince callback function
        void image_cb(const sensor_msgs::CompressedImageConstPtr& msg); // for imaging
        void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg); // for Lidar
        
        // Define manipulator function
        void updateRobotState(); // Update manipulator state
        bool setJointSpacePath(std::vector<double> joint_angle, double path_time); // Set Joint angle
        bool setToolControl(std::vector<double> joint_angle);

        
        // Define gripper of manipulator
        void pickUp();
        void pickDown();
        void sliceLeft(); // hit object from front to left
        void sliceRight(); // hit object from front to right
        void setRLabel(Mat& image, string str, vector<Point> contour); // labeling about Red
        void setYLabel(Mat& image, string str, vector<Point> contour); // labeling about Yellow

        // Define robot's movement
        void make90DegreeRight();
        void make90DegreeLeft();

        // Define pulisher for movement
        ros::Publisher vel_pub; // Publisher for control velocity
        geometry_msgs::Twist twist_msg; // Variable for publishing velocity
        
        int cols = 0; int rows = 0; // Variable for reading video

    private:
        int init_argc;
        char** init_argv;

        // Subscriber for Lidar and Video
        ros::Subscriber laser_sub;
        ros::Subscriber image_sub;

        /*
        // Variables for various situation
        */

        std::vector<float> range; // vector for Lidar sensor
        std::vector<float> range_old; // vector for old Lidar sensor
        
        // variable for filtering hsv, and find the center of circle
        int low_h_, high_h_, low_s_, high_s_, low_v_, high_v_, x_circle, y_circle;
        
        // variable for count Non-zero pixels in the mask
        int numPixelB, numPixelG, numPixelR, numPixelY, numPixelP;
        
        float fl89, fr271, br269, bl91, fl2, fr358;
        float front = 3.5, left = 3.5, right = 3.5; // variable for reading Lidar sensor
        double current_angle; double target_angle; // variable for count angle about rotation
        
        // Variable for detect colors
        bool flagB = false; bool flagG = false;
        bool flagR = false; bool flagY = false;
        bool flagP = false;

        // Variable for decide movement
        bool rotationL = false; bool rotationR = false;
        bool straight = true;

        // Variable for current state of gripper
        bool pick = false;

        // Vectors for present position or joint angle
        std::vector<double> present_joint_angle_; 
        std::vector<double> present_kinematics_position_;

        // MoveIt! package
        moveit::planning_interface::MoveGroupInterface* move_group_;
        moveit::planning_interface::MoveGroupInterface* move_group2_;
};
}
#endif // define end.
