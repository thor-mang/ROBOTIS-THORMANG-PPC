/*
 * main.cpp
 *
 *  Created on: 2016. 2. 18.
 *      Author: HJSONG
 */



#include "thormang3_walking_demo/thormang3_walking_demo.h"

ros::Subscriber demo_command_sub;


bool is_init_pose = false;

void DemoCommandCallback(const std_msgs::String::ConstPtr& msg)
{

	ROS_INFO_STREAM("[Demo]  : receive [" << msg->data << "] msg " );

    if ( msg->data == "ini_pose")
    {
        ROS_INFO("demo 1: go to initial pose");
        MoveToInitPose();
        is_init_pose = true;
        ROS_INFO("[Demo]  : please wait 5 seconds");
    }
    else if ( msg->data == "set_mode")
    {
        ROS_INFO("demo 2: set walking control mode");
    	SetCtrlModule();
    }
    else if( msg->data == "forward" )
    {
    	ROS_INFO("demo 4: forward walking");
    	WalkForward();
    }
    else if( msg->data == "backward" )
    {
    	ROS_INFO("demo 5: backward walking");
    	WalkBackWard();
    }
    else if( msg->data == "balance_on" )
    {
    	ROS_INFO("demo 3: balance enable");
    	BalanceOn();
    }
    else if( msg->data == "balance_off" )
    {
    	ROS_INFO("demo 3: balance disable");
    	BalanceOff();
    }
    else {
    	ROS_ERROR("Invalid Command!!!");
    }


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "thormang3_walking_demo");

	ROS_INFO("Robotis THORMANG3 Walking Simple Demo");

	Initialize();

	ros::NodeHandle _nh;
	demo_command_sub = _nh.subscribe("robotis/walking_demo/command", 10, DemoCommandCallback);

	ros::spin();
	return 0;
}

