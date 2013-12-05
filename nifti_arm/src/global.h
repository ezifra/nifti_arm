/*
 * global.h
 *
 *  Created on: Feb 13, 2012
 *      Author: daniel
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "ros/ros.h"
#include "math.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "nifti_arm_status.h"
#include "nifti_arm_msgs/msg_height.h"
#include "nifti_arm_msgs/msg_angle.h"
#include "stdlib.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nifti_arm_msgs/msg_current.h"
#include "nifti_arm_msgs/msg_position.h"
#include "dimensions_t.h"
#include "dynamixel_msgs/JointState.h"

extern "C"
{
#include "libepos/epos.h"
#include "libepos/position_profile.h"
#include "libepos/home.h"
}
#define PI 3.14159265
#define LOOP_RATE 30
#define IMU_PUB_RATE 20


ros::Rate * prate;

#include "tools.h"
#include "NiftiArmNode.h"
//#include "boost/thread.hpp"


#endif /* GLOBAL_H_ */
