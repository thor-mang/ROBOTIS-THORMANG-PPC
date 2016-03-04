/*
 * DirectControl.cpp
 *
 *  Created on: Feb. 18, 2016
 *      Author: sch
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <pthread.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

#include "robotis_device/Dynamixel.h"

using namespace ROBOTIS;

ros::Publisher _control_module_pub;
ros::Publisher _sync_wr_item_pub;
ros::Publisher _direct_control_pub;

pthread_t thread_8ms;
pthread_mutex_t mutex_flag  = PTHREAD_MUTEX_INITIALIZER;

void control_command_callback( const std_msgs::String& msg )
{
    Dynamixel *dxl = new Dynamixel(1, "H54-100-S500-R", 2.0);

    if( msg.data == "set_module")
    {
        ROS_INFO("set module");

        std_msgs::String _msg;
        _msg.data = "DIRECT_CONTROL_MODE";

        _control_module_pub.publish( _msg );
    }
    else if ( msg.data == "set_vel")
    {
        ROS_INFO("set goal velocity");

        robotis_controller_msgs::SyncWriteItem _item_msg;

        _item_msg.item_name = "goal_velocity";
        _item_msg.joint_name.push_back( "r_arm_sh_p1" );
        _item_msg.value.push_back( 10000 );

        _sync_wr_item_pub.publish( _item_msg );
    }
    else if ( msg.data == "set_accel")
    {
        ROS_INFO("set goal acceleration");

        robotis_controller_msgs::SyncWriteItem _item_msg;

        _item_msg.item_name = "goal_acceleration";
        _item_msg.joint_name.push_back( "r_arm_sh_p1" );
        _item_msg.value.push_back( 4 );

        _sync_wr_item_pub.publish( _item_msg );
    }
    else if ( msg.data == "set_pos")
    {
        ROS_INFO("set goal position");

        sensor_msgs::JointState _msg;

        _msg.name.push_back("r_arm_sh_p1");

        double _joint_value = 0.0; // rad
        _msg.position.push_back( dxl->ConvertRadian2Value( _joint_value ) );

        _direct_control_pub.publish( _msg );
    }
}

void* thread_8ms_proc( void* arg )
{
    ros::Rate loop_rate( 125 );

    while( ros::ok() )
    {

	}

	return 0;
}

int main( int argc , char **argv )
{
    ros::init( argc , argv , "direct_control_publisher" );
    ros::NodeHandle nh("~");

    ROS_INFO("Direct Control Publisher Ready");

    /*---------- publisher ----------*/
    _control_module_pub = nh.advertise<std_msgs::String>("/robotis/set_controller_mode", 1);
    _sync_wr_item_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 1);
    _direct_control_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 1);

    /*---------- subscriber ----------*/
    ros::Subscriber _control_command_sub = nh.subscribe("/robotis/direct_control", 1, control_command_callback);
//    ros::Subscriber _joint_state_sub = nh.subscribe("/robotis/present_joint_states", 1, present_joint_states_callback);

    /*---------- 8ms thread ----------*/

    pthread_create(&thread_8ms, NULL, thread_8ms_proc, NULL);

	ros::spin();

    return 0;
}
