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
 * thormang3_walking_demo.cpp
 *
 *  Created on: 2016. 2. 18.
 *      Author: Jay Song
 */

#include "thormang3_walking_demo/thormang3_walking_demo.h"

ros::Publisher      g_wholebody_ini_pose_pub;
ros::Publisher      g_enable_ctrl_module_pub;

ros::ServiceClient  g_get_ref_step_data_client;
ros::ServiceClient  g_add_step_data_array_client;
ros::ServiceClient  g_is_running_client;
ros::ServiceClient  g_set_balance_param_client;

ros::Subscriber     g_walking_module_status_msg_sub;


double g_start_end_time = 2.0; //sec
double g_step_time      = 1.0; //sec
double g_step_length    = 0.1; //meter
double g_body_z_swap    = 0.01; //meter
double g_foot_z_swap    = 0.1; //meter


void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  ROS_INFO_STREAM("[" << msg->module_name <<"] : " << msg->status_msg);
}

void initialize()
{
  ros::NodeHandle nh;

  g_wholebody_ini_pose_pub        = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_enable_ctrl_module_pub        = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

  g_get_ref_step_data_client      = nh.serviceClient<thormang3_walking_module_msgs::GetReferenceStepData>("/robotis/walking/get_reference_step_data");
  g_add_step_data_array_client    = nh.serviceClient<thormang3_walking_module_msgs::AddStepDataArray>("/robotis/walking/add_step_data");
  g_set_balance_param_client      = nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");
  g_is_running_client             = nh.serviceClient<thormang3_walking_module_msgs::IsRunning>("/robotis/walking/is_running");

  g_walking_module_status_msg_sub = nh.subscribe("/robotis/status", 10, walkingModuleStatusMSGCallback);
}


void moveToInitPose()
{
  std_msgs::String str_msg;
  str_msg.data = "ini_pose";

  g_wholebody_ini_pose_pub.publish( str_msg );
}

void setCtrlModule()
{
  std_msgs::String set_ctrl_mode_msg;
  set_ctrl_mode_msg.data = "walking_module";
  g_enable_ctrl_module_pub.publish( set_ctrl_mode_msg );
}

bool loadBalanceParam(thormang3_walking_module_msgs::SetBalanceParam& set_param)
{
  ros::NodeHandle _ros_node;
  std::string balance_yaml_path = "";
  balance_yaml_path = ros::package::getPath("thormang3_walking_demo") + "/data/balance_param.yaml";

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(balance_yaml_path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Failed to load balance param yaml file.");
    return false;
  }

  double cob_x_offset_m                   = doc["cob_x_offset_m"].as<double>();
  double cob_y_offset_m                   = doc["cob_y_offset_m"].as<double>();
  double hip_roll_swap_angle_rad          = doc["hip_roll_swap_angle_rad"].as<double>();
  double gyro_gain                        = doc["gyro_gain"].as<double>();
  double foot_roll_angle_gain             = doc["foot_roll_angle_gain"].as<double>();
  double foot_pitch_angle_gain            = doc["foot_pitch_angle_gain"].as<double>();
  double foot_x_force_gain                = doc["foot_x_force_gain"].as<double>();
  double foot_y_force_gain                = doc["foot_y_force_gain"].as<double>();
  double foot_z_force_gain                = doc["foot_z_force_gain"].as<double>();
  double foot_roll_torque_gain            = doc["foot_roll_torque_gain"].as<double>();
  double foot_pitch_torque_gain           = doc["foot_pitch_torque_gain"].as<double>();
  double foot_roll_angle_time_constant    = doc["foot_roll_angle_time_constant"].as<double>();
  double foot_pitch_angle_time_constant   = doc["foot_pitch_angle_time_constant"].as<double>();
  double foot_x_force_time_constant       = doc["foot_x_force_time_constant"].as<double>();
  double foot_y_force_time_constant       = doc["foot_y_force_time_constant"].as<double>();
  double foot_z_force_time_constant       = doc["foot_z_force_time_constant"].as<double>();
  double foot_roll_torque_time_constant   = doc["foot_roll_torque_time_constant"].as<double>();
  double foot_pitch_torque_time_constant  = doc["foot_pitch_torque_time_constant"].as<double>();

  set_param.request.balance_param.cob_x_offset_m                  = cob_x_offset_m;
  set_param.request.balance_param.cob_y_offset_m                  = cob_y_offset_m;
  set_param.request.balance_param.hip_roll_swap_angle_rad         = hip_roll_swap_angle_rad;
  set_param.request.balance_param.gyro_gain                       = gyro_gain;
  set_param.request.balance_param.foot_roll_angle_gain            = foot_roll_angle_gain;
  set_param.request.balance_param.foot_pitch_angle_gain           = foot_pitch_angle_gain;
  set_param.request.balance_param.foot_x_force_gain               = foot_x_force_gain;
  set_param.request.balance_param.foot_y_force_gain               = foot_y_force_gain;
  set_param.request.balance_param.foot_z_force_gain               = foot_z_force_gain;
  set_param.request.balance_param.foot_roll_torque_gain           = foot_roll_torque_gain;
  set_param.request.balance_param.foot_pitch_torque_gain          = foot_pitch_torque_gain;
  set_param.request.balance_param.foot_roll_angle_time_constant   = foot_roll_angle_time_constant;
  set_param.request.balance_param.foot_pitch_angle_time_constant  = foot_pitch_angle_time_constant;
  set_param.request.balance_param.foot_x_force_time_constant      = foot_x_force_time_constant;
  set_param.request.balance_param.foot_y_force_time_constant      = foot_y_force_time_constant;
  set_param.request.balance_param.foot_z_force_time_constant      = foot_z_force_time_constant;
  set_param.request.balance_param.foot_roll_torque_time_constant  = foot_roll_torque_time_constant;
  set_param.request.balance_param.foot_pitch_torque_time_constant = foot_pitch_torque_time_constant;

  return true;
}


void setBalanceOn()
{
  thormang3_walking_module_msgs::SetBalanceParam set_balance_param_srv;
  set_balance_param_srv.request.updating_duration =  2.0*1.0; //sec

  if(loadBalanceParam(set_balance_param_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to Load Balance YAML");
    return;
  }


  if(g_set_balance_param_client.call(set_balance_param_srv) == true)
  {
    int set_balance_param_srv_result = set_balance_param_srv.response.result;
    if( set_balance_param_srv_result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to set balance param");
    else
    {
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::TIME_CONST_IS_ZERO_OR_NEGATIVE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to set balance param ");
  }
}

void setBalanceOff()
{
  thormang3_walking_module_msgs::SetBalanceParam set_balance_param_srv;
  set_balance_param_srv.request.updating_duration                             = 1.0; //sec

  if(loadBalanceParam(set_balance_param_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to Load Balance YAML");
    return;
  }

  set_balance_param_srv.request.balance_param.hip_roll_swap_angle_rad = 0;
  set_balance_param_srv.request.balance_param.gyro_gain               = 0;
  set_balance_param_srv.request.balance_param.foot_roll_angle_gain    = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_angle_gain   = 0;
  set_balance_param_srv.request.balance_param.foot_x_force_gain       = 0;
  set_balance_param_srv.request.balance_param.foot_y_force_gain       = 0;
  set_balance_param_srv.request.balance_param.foot_z_force_gain       = 0;
  set_balance_param_srv.request.balance_param.foot_roll_torque_gain   = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_torque_gain  = 0;

  if(g_set_balance_param_client.call(set_balance_param_srv) == true)
  {
    int set_balance_param_srv_result = set_balance_param_srv.response.result;
    if( set_balance_param_srv_result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to set balance param");
    else
    {
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::TIME_CONST_IS_ZERO_OR_NEGATIVE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to set balance param ");
  }
}

void walkForward()
{
  thormang3_walking_module_msgs::GetReferenceStepData    get_ref_stp_data_srv;
  thormang3_walking_module_msgs::AddStepDataArray        add_stp_data_srv;
  thormang3_walking_module_msgs::IsRunning               is_running_srv;
  thormang3_walking_module_msgs::StepData                stp_data_msg;

  if(g_is_running_client.call(is_running_srv) == false)
  {
    ROS_ERROR("Failed to get walking_module status");
  }
  else
  {
    if(is_running_srv.response.is_running == true)
    {
      ROS_ERROR("[Demo]  : ROBOT_IS_WALKING_NOW");
      return;
    }
  }

  if(g_get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
    ROS_ERROR("Failed to get reference step data");

  stp_data_msg = get_ref_stp_data_srv.response.reference_step_data;

  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  stp_data_msg.time_data.dsp_ratio = 0.2;
  stp_data_msg.time_data.abs_step_time += g_start_end_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
  stp_data_msg.position_data.body_z_swap = 0;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);

  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING;
  stp_data_msg.time_data.abs_step_time += g_step_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  stp_data_msg.position_data.body_z_swap = g_body_z_swap;
  stp_data_msg.position_data.foot_z_swap = g_foot_z_swap;
  stp_data_msg.position_data.right_foot_pose.x += g_step_length;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);

  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING;
  stp_data_msg.time_data.abs_step_time += g_step_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  stp_data_msg.position_data.body_z_swap = g_body_z_swap;
  stp_data_msg.position_data.foot_z_swap = g_foot_z_swap;
  stp_data_msg.position_data.left_foot_pose.x += g_step_length;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);

  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  stp_data_msg.time_data.abs_step_time += g_start_end_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
  stp_data_msg.position_data.body_z_swap = 0;
  stp_data_msg.position_data.foot_z_swap = 0;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);

  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  if(g_add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to add step data array");
    else
    {
      ROS_ERROR("[Demo]  : Failed to add step data array");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
  }
}

void walkBackward()
{
  thormang3_walking_module_msgs::GetReferenceStepData    get_ref_stp_data_srv;
  thormang3_walking_module_msgs::AddStepDataArray        add_stp_data_srv;
  thormang3_walking_module_msgs::IsRunning               is_running_srv;
  thormang3_walking_module_msgs::StepData                stp_data_msg;

  if(g_is_running_client.call(is_running_srv) == false)
  {
    ROS_ERROR("Failed to get walking_module status");
  }
  else
  {
    if(is_running_srv.response.is_running == true)
    {
      ROS_ERROR("[Demo]  : ROBOT_IS_WALKING_NOW");
      return;
    }
  }

  g_get_ref_step_data_client.call(get_ref_stp_data_srv);

  stp_data_msg = get_ref_stp_data_srv.response.reference_step_data;

  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  stp_data_msg.time_data.dsp_ratio = 0.2;
  stp_data_msg.time_data.abs_step_time += g_start_end_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
  stp_data_msg.position_data.body_z_swap = 0;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);


  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING;
  stp_data_msg.time_data.abs_step_time += g_step_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING;
  stp_data_msg.position_data.body_z_swap = g_body_z_swap;
  stp_data_msg.position_data.foot_z_swap = g_foot_z_swap;
  stp_data_msg.position_data.right_foot_pose.x -= g_step_length;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);


  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING;
  stp_data_msg.time_data.abs_step_time += g_step_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING;
  stp_data_msg.position_data.body_z_swap = g_body_z_swap;
  stp_data_msg.position_data.foot_z_swap = g_foot_z_swap;
  stp_data_msg.position_data.left_foot_pose.x -= g_step_length;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);


  stp_data_msg.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  stp_data_msg.time_data.abs_step_time += g_start_end_time;
  stp_data_msg.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
  stp_data_msg.position_data.body_z_swap = 0;
  stp_data_msg.position_data.foot_z_swap = 0;
  add_stp_data_srv.request.step_data_array.push_back(stp_data_msg);


  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  if(g_add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to add step data array");
    else
    {
      ROS_ERROR("[Demo]  : Failed to add step data array");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & thormang3_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
  }
}
