/*
 * thormang3_walking_demo.h
 *
 *  Created on: 2016. 2. 18.
 *      Author: HJSONG
 */

#ifndef SRC_ROBOTIS_THORMANG_PPC_THORMANG3_WALKING_DEMO_INCLUDE_THORMANG3_WALKING_DEMO_THORMANG3_WALKING_DEMO_H_
#define SRC_ROBOTIS_THORMANG_PPC_THORMANG3_WALKING_DEMO_INCLUDE_THORMANG3_WALKING_DEMO_THORMANG3_WALKING_DEMO_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/WalkingStart.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"


#include "WalkingModuleCommon.h"



void Initialize();

void MoveToInitPose();

void SetCtrlModule();

void BalanceOn();
void BalanceOff();

void WalkForward();
void WalkBackWard();


#endif /* SRC_ROBOTIS_THORMANG_PPC_THORMANG3_WALKING_DEMO_INCLUDE_THORMANG3_WALKING_DEMO_THORMANG3_WALKING_DEMO_H_ */
