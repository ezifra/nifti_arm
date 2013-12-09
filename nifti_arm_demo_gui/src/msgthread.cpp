/*
 * msgthread.cpp
 *
 *  Created on: Feb 23, 2012
 *      Author: Daniel Robin Reuter
 */
#include "msgthread.h"

msg_thread::msg_thread() {
	n = new ros::NodeHandle;
	rate = new ros::Rate(20);
	pubHeight = n->advertise<nifti_arm_msgs::msg_height> ("/arm/height", 1);
	pubCalibrate = n->advertise<std_msgs::Empty> ("/arm/calibrate", 1);
	pubHome = n->advertise<std_msgs::Empty> ("/arm/home", 1);
	pubActivate = n->advertise<std_msgs::Empty> ("/arm/activate", 1);
	pubStop = n->advertise<std_msgs::Empty> ("/arm/stop", 1);
	pubAngle = n->advertise<nifti_arm_msgs::msg_angle> ("/arm/angle", 1);
	pubManualPosition = n->advertise<nifti_arm_msgs::msg_position> ("/arm/manualPosition",
			1);
	pubManualActivate = n->advertise<std_msgs::Bool> ("/arm/manualActivate",
			1);
	pubPan = n->advertise<std_msgs::Float64> ("/pan_controller/command",
				1);
	pubTilt = n->advertise<std_msgs::Float64> ("/tilt_controller/command",
					1);
}

msg_thread::~msg_thread() {
	ros::shutdown();
}

void msg_thread::publishManualActivate(bool status){
	std_msgs::Bool msg;
	msg.data = status;
	pubManualActivate.publish(msg);
}

void msg_thread::publishManualPosition(float shoulder, float elbow){
	nifti_arm_msgs::msg_position msg;
	msg.a = shoulder;
	msg.b = elbow;
	pubManualPosition.publish(msg);
}

void msg_thread::publishHeight(int height) {
	nifti_arm_msgs::msg_height msg;
	msg.height = height;
	pubHeight.publish(msg);
}

void msg_thread::publishAngle(float angle) {
	nifti_arm_msgs::msg_angle msg;
	msg.angle = angle;
	pubAngle.publish(msg);
}

void msg_thread::publishPan(float angle) {
	std_msgs::Float64 msg;
	msg.data = angle;
	pubPan.publish(msg);
}

void msg_thread::publishTilt(float angle) {
	std_msgs::Float64 msg;
	msg.data = angle;
	pubTilt.publish(msg);
}


void msg_thread::publishCalibrate() {
	std_msgs::Empty msg;
	pubCalibrate.publish(msg);
}

void msg_thread::publishHome() {
	std_msgs::Empty msg;
	pubHome.publish(msg);
}

void msg_thread::publishStop() {
	std_msgs::Empty msg;
	pubStop.publish(msg);
}

void msg_thread::publishActivate() {
	std_msgs::Empty msg;
	pubActivate.publish(msg);
}

void msg_thread::run() {
	while (ros::ok()) {
		rate->sleep();
		ros::spinOnce();
	}
}

