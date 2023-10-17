/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Original Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */
/* OpenCV: Ctanakul */
/* Updator: Young-Jo KIM, Ye-jun SON */

// Include header file
#include "../include/video_manipulator/node.hpp"
#include <iostream>
#include <ros/ros.h>
#include <ros/spinner.h>
#include "sensor_msgs/Imu.h"
#include <math.h>
#include "geometry_msgs/Quaternion.h"

// Define speed
#define GO 0.045
#define TURN 0.05
#define SLOW 0.023
#define STURN 0.01
#define RANGE 0.21

using namespace std;
void final::Node::move() // Define move() function in Object Node, in final namespace
{
    ros::Rate loop(50); // Set period 50Hz
    while (ros::ok()) // While node is alive
    {
        // Rename flag variables
        bool red_detect = flagR;
        bool blue_detect = flagB;
        bool green_detect = flagG;
        bool yellow_detect = flagY;
        bool purple_detect = flagP;

        // Define movement
        if(straight)
        {
            twist_msg.linear.x = GO;
            twist_msg.angular.z = 0.0;
          
            // Decide turn for heading any colors
            if(x_circle > (cols/2 + 5)&&(flagB || flagG || flagR || flagY || flagP))
            {
                twist_msg.angular.z = -1 * TURN;
            }
            if(x_circle < (cols/2 - 5)&&(flagB || flagG || flagR || flagY || flagP) && x_circle != 0)
            {
                twist_msg.angular.z = TURN;
            }
            ROS_INFO("straight");
        }
      
        if (green_detect && !pick) // Green detected, and grabbed nothing
        {

            twist_msg.linear.x = SLOW;
            twist_msg.angular.z = 0.0;
          
            rotationR = false;
            rotationL = false;
            straight = true;
          
            // Decide turn for heading green color
            if(x_circle > (cols/2 + 3))
            {
                twist_msg.angular.z = -1 * STURN;
            }
            if(x_circle < (cols/2 - 3))
            {
                twist_msg.angular.z = STURN;
            }
          
            // Close enough to grab obejct (green)
            if(front <= RANGE)
            {
                twist_msg.linear.x = 0.0;
                // Decide turn for heading green color
                if(x_circle > (cols/2 + 2))
                {
                    twist_msg.angular.z = -1 * STURN;
                }
                if(x_circle < (cols/2 - 2))
                {
                    twist_msg.angular.z = STURN;
                }
                if((x_circle > (cols/2 - 2)) && (x_circle < (cols/2 + 2)))
                {
                    pickUp();
                }
            }
        }
       
        vel_pub.publish(twist_msg); // publishing velocity
        ROS_INFO("front: %f, right: %f, left: %f, pick: %d", front, right, left, pick); // print Lidar value, and state of grapping
        ros::spinOnce(); // activate Callback function
        loop.sleep(); // Sleep for node activate periode, 20ms
    } 
}

int main(int argc, char **argv) {
    final::Node n(argc,argv); // Declare "Node" Object
    n.init(); // activate init() function of "n" object variable
    n.move(); // activate move() function of "n" object variable
    return 0;
}
