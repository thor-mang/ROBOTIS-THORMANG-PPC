/*
 * thormang3_walking_demo.cpp
 *
 *  Created on: 2016. 2. 18.
 *      Author: HJSONG
 */


#include "thormang3_walking_demo/thormang3_walking_demo.h"



ros::Publisher		wholebody_ini_pose_pub;
ros::Publisher		enable_ctrl_module_pub;

ros::ServiceClient	get_ref_step_data_client;
ros::ServiceClient	add_step_data_array_client;
ros::ServiceClient	is_running_client;

ros::ServiceClient	set_balance_param_client;

ros::Subscriber		walking_module_status_msg_sub;


double start_end_time	= 2.0; //sec
double step_time		= 1.0; //sec
double step_length		= 0.1; //meter
double body_z_swap		= 0.01; //meter
double foot_z_swap		= 0.1; //meter

void WalkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
	ROS_INFO_STREAM("[" << msg->module_name <<"] : " << msg->status_msg);
}


void Initialize()
{
	ros::NodeHandle _nh;

	wholebody_ini_pose_pub		= _nh.advertise<std_msgs::String>("robotis/base/ini_pose", 0);
	enable_ctrl_module_pub 		= _nh.advertise<std_msgs::String>("robotis/enable_ctrl_module", 0);

	get_ref_step_data_client	= _nh.serviceClient<thormang3_walking_module_msgs::GetReferenceStepData>("robotis/walking/get_reference_step_data");
	add_step_data_array_client	= _nh.serviceClient<thormang3_walking_module_msgs::AddStepDataArray>("robotis/walking/add_step_data");
	set_balance_param_client	= _nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("robotis/walking/set_balance_param");

	walking_module_status_msg_sub	= _nh.subscribe("robotis/status", 10, WalkingModuleStatusMSGCallback);

}


void MoveToInitPose()
{
	std_msgs::String _str_msg;
	_str_msg.data = "ini_pose";

	wholebody_ini_pose_pub.publish( _str_msg );
}

void SetCtrlModule()
{
    std_msgs::String _demo_msg;
    _demo_msg.data = "walking_module";
    enable_ctrl_module_pub.publish( _demo_msg );
}

bool LoadBalanceParam(thormang3_walking_module_msgs::SetBalanceParam& _set_param)
{
    ros::NodeHandle _ros_node;
    std::string _balance_yaml_path = "";
    _balance_yaml_path = ros::package::getPath("thormang3_walking_demo") + "/data/balance_param.yaml";

    YAML::Node _doc;
	try	{
		// load yaml
		_doc = YAML::LoadFile(_balance_yaml_path.c_str());
	}
	catch(const std::exception& e) {
		ROS_ERROR("Failed to load balance param yaml file.");
		return false;
	}

	double cob_x_offset_m					= _doc["cob_x_offset_m"].as<double>();
	double cob_y_offset_m					= _doc["cob_y_offset_m"].as<double>();
	double hip_roll_swap_angle_rad			= _doc["hip_roll_swap_angle_rad"].as<double>();
	double gyro_gain						= _doc["gyro_gain"].as<double>();
	double foot_roll_angle_gain				= _doc["foot_roll_angle_gain"].as<double>();
	double foot_pitch_angle_gain			= _doc["foot_pitch_angle_gain"].as<double>();
	double foot_x_force_gain				= _doc["foot_x_force_gain"].as<double>();
	double foot_y_force_gain				= _doc["foot_y_force_gain"].as<double>();
	double foot_z_force_gain				= _doc["foot_z_force_gain"].as<double>();
	double foot_roll_torque_gain			= _doc["foot_roll_torque_gain"].as<double>();
	double foot_pitch_torque_gain			= _doc["foot_pitch_torque_gain"].as<double>();
	double foot_roll_angle_time_constant	= _doc["foot_roll_angle_time_constant"].as<double>();
	double foot_pitch_angle_time_constant	= _doc["foot_pitch_angle_time_constant"].as<double>();
	double foot_x_force_time_constant		= _doc["foot_x_force_time_constant"].as<double>();
	double foot_y_force_time_constant		= _doc["foot_y_force_time_constant"].as<double>();
	double foot_z_force_time_constant		= _doc["foot_z_force_time_constant"].as<double>();
	double foot_roll_torque_time_constant	= _doc["foot_roll_torque_time_constant"].as<double>();
	double foot_pitch_torque_time_constant	= _doc["foot_pitch_torque_time_constant"].as<double>();

	_set_param.request.balance_param.cob_x_offset_m                  = cob_x_offset_m;
	_set_param.request.balance_param.cob_y_offset_m                  = cob_y_offset_m;
	_set_param.request.balance_param.hip_roll_swap_angle_rad         = hip_roll_swap_angle_rad;
	_set_param.request.balance_param.gyro_gain                       = gyro_gain;
	_set_param.request.balance_param.foot_roll_angle_gain            = foot_roll_angle_gain;
	_set_param.request.balance_param.foot_pitch_angle_gain           = foot_pitch_angle_gain;
	_set_param.request.balance_param.foot_x_force_gain               = foot_x_force_gain;
	_set_param.request.balance_param.foot_y_force_gain               = foot_y_force_gain;
	_set_param.request.balance_param.foot_z_force_gain               = foot_z_force_gain;
	_set_param.request.balance_param.foot_roll_torque_gain           = foot_roll_torque_gain;
	_set_param.request.balance_param.foot_pitch_torque_gain          = foot_pitch_torque_gain;
	_set_param.request.balance_param.foot_roll_angle_time_constant   = foot_roll_angle_time_constant;
	_set_param.request.balance_param.foot_pitch_angle_time_constant  = foot_pitch_angle_time_constant;
	_set_param.request.balance_param.foot_x_force_time_constant      = foot_x_force_time_constant;
	_set_param.request.balance_param.foot_y_force_time_constant      = foot_y_force_time_constant;
	_set_param.request.balance_param.foot_z_force_time_constant      = foot_z_force_time_constant;
	_set_param.request.balance_param.foot_roll_torque_time_constant  = foot_roll_torque_time_constant;
	_set_param.request.balance_param.foot_pitch_torque_time_constant = foot_pitch_torque_time_constant;

    return true;
}


void BalanceOn()
{
	thormang3_walking_module_msgs::SetBalanceParam _set_balance_param_srv;
	_set_balance_param_srv.request.updating_duration							 =  2.0*1.0; //sec

	if(LoadBalanceParam(_set_balance_param_srv) == false)
	{
		ROS_ERROR("[Demo]  : Failed to Load Balance YAML");
		return;
	}


	if(set_balance_param_client.call(_set_balance_param_srv) == true)
	{
		int _result = _set_balance_param_srv.response.result;
		if( _result == BALANCE_PARAM_ERR::NO_ERROR)
			ROS_INFO("[Demo]  : Succeed to set balance param");
		else {
			if(_result & BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE)
				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
			if(_result & BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED)
				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
			if(_result & BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE)
				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
		}
	}
	else
		ROS_ERROR("[Demo]  : Failed to set balance param ");
}

void BalanceOff()
{
	thormang3_walking_module_msgs::SetBalanceParam _set_balance_param_srv;
	_set_balance_param_srv.request.updating_duration							 = 1.0; //sec

	if(LoadBalanceParam(_set_balance_param_srv) == false)
	{
		ROS_ERROR("[Demo]  : Failed to Load Balance YAML");
		return;
	}

	_set_balance_param_srv.request.balance_param.hip_roll_swap_angle_rad = 0;
	_set_balance_param_srv.request.balance_param.gyro_gain               = 0;
	_set_balance_param_srv.request.balance_param.foot_roll_angle_gain    = 0;
	_set_balance_param_srv.request.balance_param.foot_pitch_angle_gain   = 0;
	_set_balance_param_srv.request.balance_param.foot_x_force_gain       = 0;
	_set_balance_param_srv.request.balance_param.foot_y_force_gain       = 0;
	_set_balance_param_srv.request.balance_param.foot_z_force_gain       = 0;
	_set_balance_param_srv.request.balance_param.foot_roll_torque_gain   = 0;
	_set_balance_param_srv.request.balance_param.foot_pitch_torque_gain  = 0;


	if(set_balance_param_client.call(_set_balance_param_srv) == true)
	{
		int _result = _set_balance_param_srv.response.result;
		if( _result == BALANCE_PARAM_ERR::NO_ERROR)
			ROS_INFO("[Demo]  : Succeed to set balance param");
		else {
			if(_result & BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE)
				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
			if(_result & BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED)
				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
			if(_result & BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE)
				ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
		}
	}
	else
		ROS_ERROR("[Demo]  : Failed to set balance param ");
}

void WalkForward()
{
	thormang3_walking_module_msgs::GetReferenceStepData	_get_ref_stp_data_srv;
	thormang3_walking_module_msgs::AddStepDataArray		_add_stp_data_srv;
	thormang3_walking_module_msgs::StepData				_stp_data_msg;


	if(get_ref_step_data_client.call(_get_ref_stp_data_srv) == false)
		ROS_ERROR("Failed to get reference step data");


	_stp_data_msg = _get_ref_stp_data_srv.response.reference_step_data;


	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalkingStarting;
	_stp_data_msg.time_data.dsp_ratio = 0.2;
	_stp_data_msg.time_data.abs_step_time += start_end_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::NFootMove;
	_stp_data_msg.position_data.body_z_swap = 0;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);

	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalking;
	_stp_data_msg.time_data.abs_step_time += step_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::RFootMove;
	_stp_data_msg.position_data.body_z_swap = body_z_swap;
	_stp_data_msg.position_data.foot_z_swap = foot_z_swap;
	_stp_data_msg.position_data.right_foot_pose.x += step_length;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);

	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalking;
	_stp_data_msg.time_data.abs_step_time += step_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::LFootMove;
	_stp_data_msg.position_data.body_z_swap = body_z_swap;
	_stp_data_msg.position_data.foot_z_swap = foot_z_swap;
	_stp_data_msg.position_data.left_foot_pose.x += step_length;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);

	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalkingEnding;
	_stp_data_msg.time_data.abs_step_time += start_end_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::NFootMove;
	_stp_data_msg.position_data.body_z_swap = 0;
	_stp_data_msg.position_data.foot_z_swap = 0;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);

	_add_stp_data_srv.request.auto_start = true;
	_add_stp_data_srv.request.remove_existing_step_data = true;

	if(add_step_data_array_client.call(_add_stp_data_srv) == true) {
		int _result = _add_stp_data_srv.response.result;
		if(_result== STEP_DATA_ERR::NO_ERROR)
			ROS_INFO("[Demo]  : Succeed to add step data array");
		else {
			ROS_ERROR("[Demo]  : Failed to add step data array");
			if(_result & STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_TIME_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
			if(_result & STEP_DATA_ERR::ROBOT_IS_WALKING_NOW)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
		}
	}
	else {
		ROS_ERROR("[Demo]  : Failed to add step data array ");
	}
}

void WalkBackWard()
{
	thormang3_walking_module_msgs::GetReferenceStepData	_get_ref_stp_data_srv;
	thormang3_walking_module_msgs::AddStepDataArray		_add_stp_data_srv;
	thormang3_walking_module_msgs::StepData				_stp_data_msg;


	get_ref_step_data_client.call(_get_ref_stp_data_srv);


	_stp_data_msg = _get_ref_stp_data_srv.response.reference_step_data;


	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalkingStarting;
	_stp_data_msg.time_data.dsp_ratio = 0.2;
	_stp_data_msg.time_data.abs_step_time += start_end_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::NFootMove;
	_stp_data_msg.position_data.body_z_swap = 0;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);


	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalking;
	_stp_data_msg.time_data.abs_step_time += step_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::RFootMove;
	_stp_data_msg.position_data.body_z_swap = body_z_swap;
	_stp_data_msg.position_data.foot_z_swap = foot_z_swap;
	_stp_data_msg.position_data.right_foot_pose.x -= step_length;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);


	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalking;
	_stp_data_msg.time_data.abs_step_time += step_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::LFootMove;
	_stp_data_msg.position_data.body_z_swap = body_z_swap;
	_stp_data_msg.position_data.foot_z_swap = foot_z_swap;
	_stp_data_msg.position_data.left_foot_pose.x -= step_length;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);


	_stp_data_msg.time_data.walking_state = WalkingStateFlag::InWalkingEnding;
	_stp_data_msg.time_data.abs_step_time += start_end_time;
	_stp_data_msg.position_data.moving_foot = MovingFootFlag::NFootMove;
	_stp_data_msg.position_data.body_z_swap = 0;
	_stp_data_msg.position_data.foot_z_swap = 0;
	_add_stp_data_srv.request.step_data_array.push_back(_stp_data_msg);


	_add_stp_data_srv.request.auto_start = true;
	_add_stp_data_srv.request.remove_existing_step_data = true;

	if(add_step_data_array_client.call(_add_stp_data_srv) == true) {
		int _result = _add_stp_data_srv.response.result;
		if(_result== STEP_DATA_ERR::NO_ERROR)
			ROS_INFO("[Demo]  : Succeed to add step data array");
		else {
			ROS_ERROR("[Demo]  : Failed to add step data array");
			if(_result & STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
			if(_result & STEP_DATA_ERR::PROBLEM_IN_TIME_DATA)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
			if(_result & STEP_DATA_ERR::ROBOT_IS_WALKING_NOW)
				ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
		}
	}
	else {
		ROS_ERROR("[Demo]  : Failed to add step data array ");
	}
}
