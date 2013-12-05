/*
 * NiftiArmNode.h
 *
 *  Created on: Mar 19, 2012
 *      Author: daniel
 */

#ifndef NIFTIARMNODE_H_
#define NIFTIARMNODE_H_

class NiftiArmNode {
public:
	NiftiArmNode();
	virtual ~NiftiArmNode();

	int initialize();
	void shutdown(void);
	void haltMotors(void);
	void floatMotors(void);
	void reactivateMotors(void);
	void calculateAngles(double &alpha, double &beta);
	void getRosParameters(void);
	void publishStatus();
	int adoptPosition(void);
	void home(void);
	void currentControl(void);
	void position_error_Control(void);
	void setNodehandle(ros::NodeHandle* nh) {
		n = nh;
		tf_br = new tf::TransformBroadcaster;
	}
	void setUpdater(diagnostic_updater::Updater* up){
		updater = up;
	}
	//Callbacks
	void onlyHeightCallback(const nifti_arm_msgs::msg_height::ConstPtr& msg);
	void angleCallback(const nifti_arm_msgs::msg_angle::ConstPtr& msg);
	void activateCallback(const std_msgs::Empty::ConstPtr& msg);
	void stopCallback(const std_msgs::Empty::ConstPtr& msg);
	void homeCallback(const std_msgs::Empty::ConstPtr& msg);
	void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void manualActivateCallback(const std_msgs::Bool::ConstPtr& msg);
	void manualPositionCallback(const nifti_arm_msgs::msg_position::ConstPtr& msg);
	void calibrateCallback(const std_msgs::Empty::ConstPtr& msg);
	void PanCallback(const dynamixel_msgs::JointState::ConstPtr& msg);
	void TiltCallback(const dynamixel_msgs::JointState::ConstPtr& msg);
	nifti_arm_status* n_status;

private:
	epos_node_t MC_1, MC_2;
	can_device_t can_dev;
	int height;
	double IMU_angle;
	double nominal_angle;
	double pan_angle;
	double tilt_angle;
	bool operating;
	bool manual;
	bool homing;
	dimensions_t dimensions;
	canSettings_t canSettings;
	ros::NodeHandle* n;
	tf::TransformBroadcaster* tf_br;
	ros::Publisher pubCurrent;
	ros::Publisher pubPosition;
	float curr1;
	float curr2;
	diagnostic_updater::Updater* updater;
	ros::Subscriber subOnlyHeight;
	ros::Subscriber subAngle;
	ros::Subscriber subActivate;
	ros::Subscriber subHome;
	ros::Subscriber subCalibrate;
	ros::Subscriber subIMU;
	ros::Subscriber subStop;
	ros::Subscriber subPan;
	ros::Subscriber subTilt;
	ros::Subscriber subManualActivate;
	ros::Subscriber subManualPosition;
};
#include "NiftiArmNode.cpp"
#endif /* NIFTIARMNODE_H_ */
