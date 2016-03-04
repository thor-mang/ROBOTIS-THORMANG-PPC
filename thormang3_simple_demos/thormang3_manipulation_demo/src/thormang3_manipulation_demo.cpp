/*
 * calc_test.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <pthread.h>

#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "robotis_controller_msgs/JointCtrlModule.h"

ros::Publisher base_ini_pose_pub;
ros::Publisher set_control_mode_msg_pub;
ros::Publisher ik_msg_pub;
ros::Publisher mani_ini_pose_pub;

void demo_command_callback( const std_msgs::String& msg )
{
    if ( msg.data == "ini_pose")
    {
        ROS_INFO("demo 1: go to initial pose");

        std_msgs::String _demo_msg;
        _demo_msg.data = "ini_pose";

        base_ini_pose_pub.publish( _demo_msg );
    }
    else if ( msg.data == "set_mode")
    {
        ROS_INFO("demo 2: set manipulation control mode");

        robotis_controller_msgs::JointCtrlModule _demo_msg;

        _demo_msg.joint_name.push_back("r_arm_sh_p1");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_sh_p1");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_sh_r");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_sh_r");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_sh_p2");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_sh_p2");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_el_y");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_el_y");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_wr_r");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_wr_r");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_wr_y");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_wr_y");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_wr_p");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_wr_p");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("torso_y");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("l_arm_grip");
        _demo_msg.module_name.push_back("manipulation_module");
        _demo_msg.joint_name.push_back("r_arm_grip");
        _demo_msg.module_name.push_back("manipulation_module");

        set_control_mode_msg_pub.publish( _demo_msg );
    }
    else if ( msg.data == "base_pose")
    {
        ROS_INFO("demo 3: go to manipulation base pose");

        std_msgs::String _demo_msgs;

        _demo_msgs.data = "ini_pose";

        mani_ini_pose_pub.publish( _demo_msgs );
    }
    else if ( msg.data == "right_arm" )
    {
        ROS_INFO("demo 4: move right arm");

        thormang3_manipulation_module_msgs::KinematicsPose _demo_msgs;

        _demo_msgs.name = "right_arm";
        _demo_msgs.pose.position.x = 0.4;
        _demo_msgs.pose.position.y = -0.2;
        _demo_msgs.pose.position.z = 0.9;
        _demo_msgs.pose.orientation.w = 1.0;
        _demo_msgs.pose.orientation.x = 0.0;
        _demo_msgs.pose.orientation.y = 0.0;
        _demo_msgs.pose.orientation.z = 0.0;

        ik_msg_pub.publish( _demo_msgs );
    }
    else if ( msg.data == "left_arm" )
    {
        ROS_INFO("demo 5: move left arm");

        thormang3_manipulation_module_msgs::KinematicsPose _demo_msgs;

        _demo_msgs.name = "left_arm_with_torso";
        _demo_msgs.pose.position.x = 0.4;
        _demo_msgs.pose.position.y = 0.2;
        _demo_msgs.pose.position.z = 0.9;
        _demo_msgs.pose.orientation.w = 1.0;
        _demo_msgs.pose.orientation.x = 0.0;
        _demo_msgs.pose.orientation.y = 0.0;
        _demo_msgs.pose.orientation.z = 0.0;

        ik_msg_pub.publish( _demo_msgs );
    }
    else
    {
        ROS_INFO("there is no demo");
    }
}



int main( int argc , char **argv )
{
    ros::init( argc , argv , "manipulation_demo_publisher" );
    ros::NodeHandle nh("~");

    base_ini_pose_pub  = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    set_control_mode_msg_pub = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 0);
    ik_msg_pub  =   nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/manipulation/ik_msg", 0);
    mani_ini_pose_pub = nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);

    ros::Subscriber demo_command_sub = nh.subscribe("/robotis/manipulation_demo/command", 5, demo_command_callback);

    ROS_INFO("Robotis Thormang3 Manipulation Simple Demo");

	ros::spin();

    return 0;
}
