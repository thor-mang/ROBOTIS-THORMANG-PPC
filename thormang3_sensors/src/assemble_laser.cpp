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

ros::Time           g_lidar_move_start_time;

ros::Publisher      g_point_cloud2_pub;

ros::Subscriber     g_lidar_turn_start_sub;
ros::Subscriber     g_lidar_turn_end_sub;

ros::ServiceClient  g_assemble_chest_laser_client;

typedef pcl::PointXYZ PointT;

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void assembleLaserScans(ros::Time before_time, ros::Time end_time)
{
  ros::Time now = ros::Time::now();

  laser_assembler::AssembleScans2 service;
  service.request.begin = before_time;
  service.request.end = end_time;

  if (g_assemble_chest_laser_client.call(service))
  {
    ros::Time assemble_time = ros::Time::now();
    sensor_msgs::PointCloud2 assembler_output = service.response.cloud;
    if (assembler_output.data.size() == 0)
    {
      // ROS_INFO("No scan data");
      return;
    }

    ROS_INFO("  ---  publish pointcloud data!!  ---  %f", (ros::Time::now() - assemble_time).toSec());

    g_point_cloud2_pub.publish(assembler_output);
  }
}

void lidarTurnCallBack(const std_msgs::String::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  if (msg->data == "start")
  {
    g_lidar_move_start_time = now;
  }
  else if (msg->data == "end")
  {
    // assemble laser
    assembleLaserScans(g_lidar_move_start_time, now);
    g_lidar_move_start_time = now;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thor_lidar_assembler");
  ros::NodeHandle nh;

  // Add your ros communications here.
  g_lidar_turn_end_sub          = nh.subscribe("/robotis/sensor/move_lidar", 1, &lidarTurnCallBack);
  g_point_cloud2_pub            = nh.advertise<sensor_msgs::PointCloud2>("/robotis/sensor/assembled_scan", 0);
  g_assemble_chest_laser_client = nh.serviceClient<laser_assembler::AssembleScans2>("/robotis/sensor/service/assemble_scans2");

  g_lidar_move_start_time = ros::Time::now();

  ros::spin();

  return 0;
}
