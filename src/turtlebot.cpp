/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file turtlebot.cpp
*@author Sudarshan Raghunathan
*@brief  Functions definitions for turtlebot class
*/

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlebot.hpp"
#include "line_follower_turtlebot/pos.h"

void turtlebot::dir_sub(line_follower_turtlebot::pos msg) {
    turtlebot::dir = msg.direction;
}

void turtlebot::vel_cmd(geometry_msgs::Twist &velocity,
 ros::Publisher &pub, ros::Rate &rate) {

    // If robot has to search
    if (turtlebot::dir == -1) {
        turtlebot::last_dir ==0;
        velocity.linear.x = 0;
        velocity.angular.z = 0.40;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
        
    } else {
        double max_vel=0.12;
        double Kp=0.0025;
        double Kd=0.007;
        double angular_z = 1000*(Kp *turtlebot::dir + Kd * (turtlebot::dir - turtlebot::last_dir));     //in turtlebot_autorace the error goes from -500 to 500 so we multiply by 1000
        
        turtlebot::last_dir=turtlebot::dir;
        
        velocity.linear.x = min(max_vel * ((1 - abs(error) / 0.05) ** 2.2), 0.2);
        velocity.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Tracking! Error is: " << turtlebot::dir << " and last error was " << turtlebot::last_dir << endl);
        ROS_INFO_STREAM("Linear velocity is: " << velocity.linear << endl);
    }
}
