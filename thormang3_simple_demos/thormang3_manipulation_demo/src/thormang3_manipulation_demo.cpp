/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * thormang3_manipulation_demo.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pthread.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"

ros::Publisher g_base_ini_pose_pub;
ros::Publisher g_enable_ctrl_module_pub;
ros::Publisher g_kinematics_msg_pub;
ros::Publisher g_manipulation_ini_pose_pub;

void demoCommandCallback(const std_msgs::String& msg)
{
  if (msg.data == "ini_pose")
  {
    ROS_INFO("demo 1: go to initial pose");

    std_msgs::String demo_msg;
    demo_msg.data = "ini_pose";

    g_base_ini_pose_pub.publish(demo_msg);
  }
  else if (msg.data == "set_mode")
  {
    ROS_INFO("demo 2: set manipulation control mode");

    std_msgs::String demo_msg;
    demo_msg.data = "manipulation_module";

    g_enable_ctrl_module_pub.publish(demo_msg);
  }
  else if (msg.data == "base_pose")
  {
    ROS_INFO("demo 3: go to manipulation base pose");

    std_msgs::String demo_msgs;
    demo_msgs.data = "ini_pose";

    g_manipulation_ini_pose_pub.publish(demo_msgs);
  }
  else if (msg.data == "right_arm")
  {
    ROS_INFO("demo 4: move right arm");

    thormang3_manipulation_module_msgs::KinematicsPose demo_msgs;

    demo_msgs.name = "right_arm";
    demo_msgs.pose.position.x = 0.4;
    demo_msgs.pose.position.y = -0.2;
    demo_msgs.pose.position.z = 0.9;
    demo_msgs.pose.orientation.w = 1.0;
    demo_msgs.pose.orientation.x = 0.0;
    demo_msgs.pose.orientation.y = 0.0;
    demo_msgs.pose.orientation.z = 0.0;

    g_kinematics_msg_pub.publish(demo_msgs);
  }
  else if (msg.data == "left_arm")
  {
    ROS_INFO("demo 5: move left arm");

    thormang3_manipulation_module_msgs::KinematicsPose demo_msgs;

    demo_msgs.name = "left_arm_with_torso";
    demo_msgs.pose.position.x = 0.4;
    demo_msgs.pose.position.y = 0.2;
    demo_msgs.pose.position.z = 0.9;
    demo_msgs.pose.orientation.w = 1.0;
    demo_msgs.pose.orientation.x = 0.0;
    demo_msgs.pose.orientation.y = 0.0;
    demo_msgs.pose.orientation.z = 0.0;

    g_kinematics_msg_pub.publish(demo_msgs);
  }
  else
  {
    ROS_INFO("there is no demo");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_demo_publisher");
  ros::NodeHandle nh("~");

  g_base_ini_pose_pub         = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_enable_ctrl_module_pub    = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  g_kinematics_msg_pub        = nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/manipulation/kinematics_pose_msg", 0);
  g_manipulation_ini_pose_pub = nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);

  ros::Subscriber demo_command_sub = nh.subscribe("/robotis/manipulation_demo/command", 5, demoCommandCallback);

  ROS_INFO("ROBOTIS THORMANG3 Manipulation Simple Demo");

  ros::spin();

  return 0;
}
