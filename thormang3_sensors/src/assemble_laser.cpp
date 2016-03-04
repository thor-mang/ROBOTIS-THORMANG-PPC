
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <laser_assembler/AssembleScans2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Time lidar_move_start_time;

ros::Subscriber lidar_turn_start_sub;
ros::Subscriber lidar_turn_end_sub;
ros::Publisher pointCloud2_pub;
ros::ServiceClient assemble_chest_laser_Client;

typedef pcl::PointXYZ PointT;

/*****************************************************************************
** Implementation
*****************************************************************************/

void assembleLaserScans(ros::Time beforeTime_, ros::Time endTime_)
{
    ros::Time _now = ros::Time::now();

    laser_assembler::AssembleScans2 _service;
    _service.request.begin = beforeTime_;
    _service.request.end = endTime_;

    if (assemble_chest_laser_Client.call(_service))
    {
        ros::Time _assembleTime = ros::Time::now();
        sensor_msgs::PointCloud2 assemblerOutput = _service.response.cloud;
        if (assemblerOutput.data.size() == 0)
        {
            // ROS_INFO("No scan data");
            return;
        }

        ROS_INFO("  ---  publish pointcloud data!!  ---  %f", (ros::Time::now() - _assembleTime).toSec());
        
        pointCloud2_pub.publish(assemblerOutput);
    }
}

void lidarTurnCallBack(const std_msgs::String::ConstPtr& msg)
{
    ros::Time _now = ros::Time::now();

    if(msg->data == "start")
    {
        lidar_move_start_time = _now;
    }
    else if(msg->data == "end")
    {
        // assemble laser
        assembleLaserScans(lidar_move_start_time, _now);
        lidar_move_start_time = _now;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thor_lidar_assembler");
    ros::NodeHandle _nh;

    // Add your ros communications here.
    lidar_turn_end_sub = _nh.subscribe("/robotis/sensor/move_lidar", 1, &lidarTurnCallBack);
    pointCloud2_pub = _nh.advertise<sensor_msgs::PointCloud2>("/robotis/sensor/assembled_scan", 0);
    assemble_chest_laser_Client = _nh.serviceClient<laser_assembler::AssembleScans2>("/robotis/sensor/service/assemble_scans2");

    lidar_move_start_time = ros::Time::now();

    ros::spin();

    return 0;
}
