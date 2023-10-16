#include <ros/ros.h>
#include <ros/network.h>
#include <iostream>

#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include "../include/video_manipulator/node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>

#define GRIP_OPEN  0.0125
#define GRIP_CLOSE –0.01

using namespace std;
using namespace cv;

namespace final {

// Node Object Constructor
Node::Node(int argc, char** argv) 
    :init_argc(argc),
    init_argv(argv)
{}

// Node Object Destructor
Node::~Node()
{
    if(ros::isStarted()) // When node is activated
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
}

bool Node::init()
{    ros::init(init_argc,init_argv,"video_manipulator"); // Activate “video_manipulator” Node

    if ( ! ros::master::check() ) // Check for parameter server
    {
        return false;
    }

    ros::start(); // Explicitly set for Node handler
    ros::NodeHandle nh_; // Activate Nodehandler
   
    // For activate manipulator
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    // Define "arm" as Move Group
    std::string planning_group_name = "arm";
    move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
    // Define "gripper" as Move Group
    std::string planning_group_name2 = "gripper";
    move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);

    // Subscribing compressed image
    image_sub = nh_.subscribe("/camera/image/compressed", 1, &Node::image_cb, this);
    // Subscribing Lidar sensor
    laser_sub = nh_.subscribe("/scan", 1,&Node::laser_cb, this);
    // Publishing velocity
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    // Initial value for HSV filtering
    low_h_ = 163;
    high_h_ = 179;
    low_s_ = 204;
    high_s_ = 255;
    low_v_ = 103;
    high_v_ = 147;

    // Trackbar for user filtering
    cv::namedWindow("thresh_img", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("low_h_", "thresh_img", &low_h_, 179);
    cv::createTrackbar("high_h_", "thresh_img", &high_h_, 179);
    cv::createTrackbar("low_s_", "thresh_img", &low_s_, 255);
    cv::createTrackbar("high_s_", "thresh_img", &high_s_, 255);
    cv::createTrackbar("low_v_", "thresh_img", &low_v_, 255);
    cv::createTrackbar("high_v_", "thresh_img", &high_v_, 255);
    
    return true;
}

void Node::updateRobotState()
{
    ros::AsyncSpinner spinner(1); 
    spinner.start();
    std::vector<double> jointValues = move_group_->getCurrentJointValues();
    std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();

    std::vector<double> temp_angle;
    temp_angle.push_back(jointValues.at(0));
    temp_angle.push_back(jointValues.at(1));
    temp_angle.push_back(jointValues.at(2));
    temp_angle.push_back(jointValues.at(3));
    temp_angle.push_back(jointValues2.at(0));
    present_joint_angle_ = temp_angle;
    geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose;  

    std::vector<double> temp_position;
    temp_position.push_back(current_pose.position.x);
    temp_position.push_back(current_pose.position.y);
    temp_position.push_back(current_pose.position.z);
    present_kinematics_position_ = temp_position;
}

// return joint angle
std::vector<double> Node::getPresentJointAngle()
{
    return present_joint_angle_;
}

// return position of end-effector
std::vector<double> Node::getPresentKinematicsPosition()
{
    return present_kinematics_position_;
}

// change the joint angle of manipulator
bool Node::setJointSpacePath(std::vector<double> joint_angle, double path_time)
{
    ros::AsyncSpinner spinner(1); 
    spinner.start(); // activate spinner

    // Next get the current set of joint values for the group.
    const robot_state::JointModelGroup* joint_model_group =
      move_group_->getCurrentState()->getJointModelGroup("arm");
    // Get current state from "arm" Move group
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

    // Copy current joint angle
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = joint_angle.at(0);  
    joint_group_positions[1] = joint_angle.at(1); 
    joint_group_positions[2] = joint_angle.at(2); 
    joint_group_positions[3] = joint_angle.at(3); // Store received joint angle

    // Store target joint angle by moveit package
    move_group_->setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Store ability for moving joint
    bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success == false)
      return false;

    move_group_->move(); // Move manipulator
    spinner.stop(); // Stop Async Spinner

    ros::Duration(path_time).sleep(); // Wait for moving time
    return true;
}

// Change gripper angle
bool Node::setToolControl(std::vector<double> joint_angle)
{
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    const robot_state::JointModelGroup* joint_model_group =
      move_group2_->getCurrentState()->getJointModelGroup("gripper");
    moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = joint_angle.at(0);
    move_group2_->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group2_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success == false)
      return false;
    move_group2_->move();
    spinner.stop();
    return true;
}


void Node::image_cb(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // Copy image
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what()); // Display Error
        return;
    }

    // Define image Variables
    cv::Mat imHSV;
    cv::Mat mask, maskB, maskG, maskR, maskY, maskP;
    cv::Mat image, gray, img_circle;

    cv::cvtColor(cv_ptr->image, imHSV, CV_BGR2HSV); // Convert color to HSV

    // Set HSV filter
    std::vector<int> low_hsv = {low_h_, low_s_, low_v_};
    std::vector<int> high_hsv = {high_h_, high_s_, high_v_};
    std::vector<int> red_low_hsv = {140, 80, 82};
    std::vector<int> red_high_hsv = {179, 233, 207};
    std::vector<int> green_low_hsv = {47, 57, 80};
    std::vector<int> green_high_hsv = {100, 163, 144};
    std::vector<int> blue_low_hsv = {78, 123, 70};
    std::vector<int> blue_high_hsv = {112, 232, 159};
    std::vector<int> purple_low_hsv = {122, 85, 53};
    std::vector<int> purple_high_hsv = {160, 255, 255};
    std::vector<int> yellow_low_hsv = {9, 90, 97};
    std::vector<int> yellow_high_hsv = {52, 150, 175};
    
    // Save mask with filtering
    cv::inRange(imHSV, low_hsv, high_hsv, mask);
    cv::inRange(imHSV, blue_low_hsv, blue_high_hsv, maskB);
    cv::inRange(imHSV, green_low_hsv, green_high_hsv, maskG);
    cv::inRange(imHSV, red_low_hsv, red_high_hsv, maskR);
    cv::inRange(imHSV, yellow_low_hsv, yellow_high_hsv, maskY);
    cv::inRange(imHSV, purple_low_hsv, purple_high_hsv, maskP);
    
    // Bitwise with original and each masks: Find colors
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, image, maskG);
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, image, maskB);
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, image, maskR);
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, image, maskY);
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, image, maskP);

    img_circle = cv_ptr->image.clone(); // Copy original image
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); // Convert color to GRAY
    std::vector<cv::Vec3f> circles; // Define Vector for information of circle
    // Find Circles from GRAY image
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/32, 100, 15, 0, 0);

    cols = gray.cols; rows = gray.rows; // saving information of GRAY image

    for (size_t i = 0; i < circles.size(); i++) // About every circles
    {
        cv::Vec3i c = circles[i];
        cv::Point center(c[0], c[1]);
        x_circle = c[0]; y_circle = c[1];
        int radius = c[2];
        cv::circle(img_circle, center, radius, cv::Scalar(200,100,100), 3);
        // print Circles in image_circle
        if(i==2) break; // break when numbers of circles is over 3
    }

    // Count numbers of non-zero pixels about each color
    numPixelB = cv::countNonZero(maskB);
    numPixelG = cv::countNonZero(maskG);
    numPixelR = cv::countNonZero(maskR);
    numPixelY = cv::countNonZero(maskY);
    numPixelP = cv::countNonZero(maskP);

    // Define flag for detecting Color
    flagB = (numPixelB > 20) ? true : false;
    flagG = (numPixelG > 20) ? true : false;
    flagR = (numPixelR > 20) ? true : false;
    flagY = (numPixelY > 20) ? true : false;
    flagP = (numPixelP > 20) ? true : false;

    // Pring image
    cv::imshow("Original", cv_ptr->image); // Original
    cv::imshow("Circles", img_circle); // Circles
    cv::imshow("Mask", mask); // Custom filter
    cv::imshow("Mask Y", maskY);
    cv::imshow("Mask B", maskB);
    cv::imshow("Mask G", maskG);
    cv::imshow("Mask R", maskR);
    cv::imshow("Mask P", maskP);

    cv::waitKey(1);
}

void Node::make90DegreeLeft()
{
}

void Node::make90DegreeRight()
{
}

void Node::laser_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    range = msg->ranges; // Copy data
    for(int i=0; i<range.size(); i++) // for every value
    {
        if(std::isinf(range[i])) range[i] = 3.5; // save 3.5 for 'inf'
    }  

    front = range[0];
    // Lidar data based on clockwised azimuth
    left = range[59]; // 90 degree
    right = range[177]; // 270 degree
    fl89 = range[58]; // 88.5 degree
    fr271 = range[178]; // 271.5 degree
    br269 = range[176]; // 268.5 degree
    bl91 = range[60]; // 91.5 degree
    fl2 = range[2]; // 3 degree
    fr358 = range[233]; // 357 degree
}

void Node::pickUp()
{
    twist_msg.linear.x = 0; 
    twist_msg.angular.z = 0;
    vel_pub.publish(twist_msg);

    std::vector<double> joint_angle;
    std::vector<double> grip_angle;
    for(size_t i=0; i<4; i++)
      joint_angle.push_back(0.0);
    grip_angle.push_back(0.0);

    double path_time = 7.0;
    ROS_INFO("get Order: to pick up");

    joint_angle[0] = 0;
    joint_angle[1] = 0.6;
    joint_angle[2] = 0.4;
    joint_angle[3] = -0.9;
    grip_angle[0] = GRIP_OPEN;

    if(!setToolControl(grip_angle))
    {
    ROS_INFO("Failed to move");
    return;
    }
    ros::Duration(3).sleep();

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    grip_angle[0] = GRIP_CLOSE;

    if(!setToolControl(grip_angle))
    {
    ROS_INFO("Failed to move");
    return;
    }
    ros::Duration(3).sleep();

    joint_angle[0] = 0;
    joint_angle[1] = -1.2;
    joint_angle[2] = 0.4;
    joint_angle[3] = 0.8;

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    pick = true;
}


void Node::pickDown()
{
    twist_msg.angular.z = 0;
    twist_msg.linear.x = 0;
    vel_pub.publish(twist_msg);

    std::vector<double> joint_angle;
    std::vector<double> grip_angle;
    for(size_t i=0; i<4; i++)
      joint_angle.push_back(0.0);
    grip_angle.push_back(0.0);

    double path_time = 7.0;
    ROS_INFO("get Order: to pick down");

    joint_angle[0] = 0;
    joint_angle[1] = 0.6;
    joint_angle[2] = 0.4;
    joint_angle[3] = -0.9;
    grip_angle[0] = GRIP_CLOSE;

    if(!setToolControl(grip_angle))
    {
    ROS_INFO("Failed to move");
    return;
    }
    ros::Duration(3).sleep();

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    grip_angle[0] = GRIP_OPEN;
    if(!setToolControl(grip_angle))
    {
    ROS_INFO("Failed to move");
    return;
    }
    ros::Duration(3).sleep();

    joint_angle[0] = 0;
    joint_angle[1] = -1.2;
    joint_angle[2] = 0.4;
    joint_angle[3] = 0.8;

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    pick = false;
}

void Node::sliceRight()
{
    twist_msg.linear.x = 0; 
    twist_msg.angular.z = 0;
    vel_pub.publish(twist_msg);

    std::vector<double> joint_angle;
    std::vector<double> grip_angle;
    for(size_t i=0; i<4; i++)
      joint_angle.push_back(0.0);
    grip_angle.push_back(0.0);

    double path_time = 7.0;

    ROS_INFO("get Order: to slice Right");

    joint_angle[0] = 0.0;
    joint_angle[1] = 0.6;
    joint_angle[2] = 0.4;
    joint_angle[3] = -0.8;
    grip_angle[0] = GRIP_CLOSE;

    if(!setToolControl(grip_angle))
    {
    ROS_INFO("Failed to move");
    return;
    }
    ros::Duration(3).sleep();

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    joint_angle[0] = -1.2;
    joint_angle[1] = 0.6;
    joint_angle[2] = 0.4;
    joint_angle[3] = -0.8;

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    joint_angle[0] = 0;
    joint_angle[1] = -1.2;
    joint_angle[2] = 0.4;
    joint_angle[3] = 0.8;

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

}

void Node::sliceLeft()
{
    twist_msg.linear.x = 0; 
    twist_msg.angular.z = 0;
    vel_pub.publish(twist_msg);

    std::vector<double> joint_angle;
    std::vector<double> grip_angle;
    for(size_t i=0; i<4; i++)
      joint_angle.push_back(0.0);
    grip_angle.push_back(0.0);

    double path_time = 7.0;

    ROS_INFO("get Order: to slice Left");

    joint_angle[0] = 0.0;
    joint_angle[1] = 0.6;
    joint_angle[2] = 0.4;
    joint_angle[3] = -0.8;
    grip_angle[0] = GRIP_CLOSE;

    if(!setToolControl(grip_angle))
    {
    ROS_INFO("Failed to move");
    return;
    }
    ros::Duration(3).sleep();
    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    joint_angle[0] = 1.2;
    joint_angle[1] = 0.6;
    joint_angle[2] = 0.4;
    joint_angle[3] = -0.8;

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

    joint_angle[0] = 0;
    joint_angle[1] = -1.2;
    joint_angle[2] = 0.4;
    joint_angle[3] = 0.8;

    if(!setJointSpacePath(joint_angle, path_time))
    {
    ROS_INFO("Failed to move");
    return;
    }

}}
