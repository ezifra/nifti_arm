/*
 * motorcontroller.h
 *
 *  Created on: Feb 13, 2012
 *      Author: daniel
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#define ELMO_CURRENT_INDEX_ACTUAL_VALUE 0x6078
#define ELMO_POSITION_ERROR_INDEX_VALUE 0x20A1
#define ELMO_CONTROLWORD 0x6040
#define ELMO_STATUSWORD 0x6041
#define ELMO_HOME_OFFSET 0x607C
#define ELMO_DISPLAY_MODE_OF_OPERATION 0x6061
#define ELMO_HOMING_METHOD 0x6098
#define ELMO_HOMING_SPEEDS 0x6099
#define ELMO_HOMING_SPEEDS_SWITCH 1
#define ELMO_HOMING_SPEEDS_INDEX_PULSE 2
#define ELMO_HOMING_ACCELERATION 0x609A
#define ELMO_BITMASK_HOMING_ATTAINED 4096
#define ELMO_MODE_OF_OPERATION 0x6060
#define ELMO_MODE_HOMING 6
#define ELMO_HOMING_METHOD_POSITIVE_LIMIT_SWITCH_AND_INDEX_PULSE 2
#define ELMO_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH_AND_INDEX_PULSE 1
#define ELMO_HOMING_METHOD_CURRENT_POSITION 35
#define ELMO_HOMING_METHOD_NEG_HOME_SWITCH_INDEX_LEFT 5
#define ELMO_HOMING_METHOD_NEG_HOME_SWITCH_INDEX_RIGHT 6
#define ELMO_HOMING_METHOD_NEG_HOME_SWITCH_LEFT 21
#define ELMO_HOMING_METHOD_NEG_HOME_SWITCH_RIGHT 22
#define ELMO_BITMASK_TARGET_REACHED 1024

int initializeMotorControllerDevice(epos_node_p mc, int id,
		can_device_t* can_dev);
float elmo_get_current(epos_device_p dev);
float elmo_get_position_error(epos_node_p node);
void elmo_start_homing (epos_node_p node, float speed, unsigned int acceleration, int datum, unsigned char method);
void elmo_wait_homed(epos_device_p dev, ros::Rate *rate, NiftiArmNode * arm);
void elmo_wait_target_reached(epos_device_p dev, ros::Rate *rate, NiftiArmNode * arm);


#include "NiftiArmNode.h"

int initializeMotorControllerDevice(epos_node_p mc, int id,
		can_device_t* can_dev) {
	int error = 0;
	ROS_INFO("Initializing Motor Controller %d", id);
	config_t config;
	param_t param;
	//Epos-Config
	config_init(&config);
	param_init_int(&param, "node-id", id);
	config_set_param(&config, &param);
	param_init_int(&param, "enc-pulse", 2000);
	config_set_param(&config, &param);
	param_init_int(&param, "control-type", 3);
	config_set_param(&config, &param);
	param_init_int(&param, "enc-type", 3);
	config_set_param(&config, &param);
	param_init_int(&param, "motor-type", 10);
	config_set_param(&config, &param);
	param_init_int(&param, "motor-current", 1);
	config_set_param(&config, &param);
	epos_init(mc, can_dev, &config);
	config_destroy(&config);
	printf("Opening node... ");
	fflush(stdout);
	error = epos_open(mc);
	if (error) {
		ROS_ERROR("error (%d).\n", error);
		return error;
	} else
		printf("OK.\n");
	mc->dev.node_id = id;
	mc->gear.sensor->num_pulses = 2048; //nominell 2000
	mc->gear.transmission = 100.0;
	return 0;
}

float elmo_get_current(epos_device_p dev) {
	short current;
	epos_device_read(dev, ELMO_CURRENT_INDEX_ACTUAL_VALUE, 0, (unsigned char*)&current, sizeof(short));

	return ((float) current) / 1000.0;
}

float elmo_get_position_error(epos_node_p node) {
	int position;
	epos_device_read(&node->dev, ELMO_POSITION_ERROR_INDEX_VALUE, 0, (unsigned char*)&position, sizeof(int));

	return epos_gear_to_angle(&node->gear, position);
}

void elmo_start_homing (epos_node_p node, float speed, unsigned int acceleration, int datum, unsigned char method) {
	epos_control_set_type(&node->control, epos_control_homing);
	epos_device_write(&node->dev, EPOS_HOME_INDEX_METHOD, 0, &method, 1);
	//epos_home_set_method(&node->dev, method);
	epos_home_set_offset(&node->dev, datum);
	epos_home_set_switch_search_velocity(&node->dev, abs(epos_gear_from_angular_velocity(&node->gear,
							speed)));
	epos_home_set_zero_search_velocity(&node->dev, abs(epos_gear_from_angular_velocity(&node->gear,
							speed)));
	epos_home_set_acceleration(&node->dev, acceleration);
	//epos_home_set_position(&node->dev, 1000);
	//epos_profile_set_type(&node->dev, epos_profile_linear);
	epos_device_set_control(&node->dev, EPOS_DEVICE_CONTROL_SHUTDOWN);
	epos_control_start(&node->control);
	epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_START);

	epos_control_start(&node->control);
	epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_START);
}

void elmo_wait_homed(epos_device_p dev, ros::Rate *rate, NiftiArmNode * arm) {
	while (!(EPOS_HOME_STATUS_REACHED & epos_device_get_status(dev))
			&& ros::ok()) {
		rate->sleep();
		arm->position_error_Control();
		arm->publishStatus();
		//ros::spinOnce();
	}
}

void elmo_wait_target_reached(epos_device_p dev, ros::Rate *rate, NiftiArmNode * arm) {
	while (!(epos_device_get_status(dev) & ELMO_BITMASK_TARGET_REACHED) && ros::ok()) {
		rate->sleep();
		arm->position_error_Control();
		arm->publishStatus();
		//ros::spinOnce();
	}
}

#endif /* MOTORCONTROLLER_H_ */
