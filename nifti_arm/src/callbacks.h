/*
 * callbacks.h
 *
 *  Created on: Mar 22, 2012
 *      Author: daniel
 */

#ifndef CALLBACKS_H_
#define CALLBACKS_H_


void NiftiArmNode::onlyHeightCallback(const nifti_arm_msgs::msg_height::ConstPtr& msg) {
	height = (double) msg->height - dimensions.mounting_height
			- dimensions.ptu_height - dimensions.ptu_height_camera;
	adoptPosition();
}

void NiftiArmNode::PanCallback(const dynamixel_msgs::JointState::ConstPtr& msg) {
	pan_angle = msg->current_pos;
	n_status->position_pan = pan_angle;
	/*tf::Transform tf_ptu_joint;
	tf_ptu_joint.setRotation(tf::createQuaternionFromRPY(0, tilt_angle,
			pan_angle));
	tf_ptu_joint.setOrigin(tf::Vector3(0, 0, dimensions.ptu_height / 1000));
	tf_br->sendTransform(tf::StampedTransform(tf_ptu_joint, ros::Time::now(),
			"hand", "ptu_joint"));*/

}

void NiftiArmNode::TiltCallback(const dynamixel_msgs::JointState::ConstPtr& msg) {
	tilt_angle = msg->current_pos;
	n_status->position_tilt = tilt_angle;
	/*tf::Transform tf_ptu_joint;
	tf_ptu_joint.setRotation(tf::createQuaternionFromRPY(0, tilt_angle,
			pan_angle));
	tf_ptu_joint.setOrigin(tf::Vector3(0, 0, dimensions.ptu_height / 1000));
	tf_br->sendTransform(tf::StampedTransform(tf_ptu_joint, ros::Time::now(),
			"hand", "ptu_joint"));*/
}

void NiftiArmNode::angleCallback(const nifti_arm_msgs::msg_angle::ConstPtr& msg) {
	nominal_angle = msg->angle;
	adoptPosition();
}

void NiftiArmNode::activateCallback(const std_msgs::Empty::ConstPtr& msg) {
	operating = true;
	n_status->status = Ok;
}

void NiftiArmNode::homeCallback(const std_msgs::Empty::ConstPtr& msg) {
	this->home();
}

void NiftiArmNode::stopCallback(const std_msgs::Empty::ConstPtr& msg) {
	operating = false;
	epos_device_shutdown(&MC_1.dev);
	epos_device_shutdown(&MC_2.dev);
}

void NiftiArmNode::manualActivateCallback(const std_msgs::Bool::ConstPtr& msg) {
	manual = msg->data;
}
void NiftiArmNode::manualPositionCallback(
		const nifti_arm_msgs::msg_position::ConstPtr& msg) {
	if (manual) {
		epos_position_profile_t profileMC_shoulder, profileMC_elbow;
		//Starting Profile
		epos_position_profile_init(&profileMC_shoulder, (msg->a
				+ dimensions.home_angle_shoulder), dimensions.max_vel / 3,
				(float) (490000 / 360 * PI), (float) (490000 / 360 * PI),
				epos_profile_linear);
		profileMC_shoulder.relative = 0;
		epos_position_profile_init(&profileMC_elbow, (msg->b
				+ dimensions.home_angle_elbow), dimensions.max_vel / 3,
				(float) (490000.0 / 360.0 * PI), (float) (490000.0 / 360 * PI),
				epos_profile_linear);
		profileMC_elbow.relative = 0;
		epos_position_profile_start(&MC_1, &profileMC_shoulder);
		epos_position_profile_start(&MC_2, &profileMC_elbow);
	}
}

void NiftiArmNode::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	static int counter;
	counter++;
	if (!(counter % (IMU_PUB_RATE / 2))) {
		tf::Quaternion q;
		tf::quaternionMsgToTF(msg->orientation, q);

		#if ROS_VERSION_MINIMUM(1,8,0) //Minimum is Fuerte
		  tf::Matrix3x3 matrix(q);
                  tfScalar roll, pitch, yaw;
                  matrix.getEulerYPR(yaw, pitch, roll);
		#else
                  btMatrix3x3 matrix(q);
                  btScalar roll, pitch, yaw;
                  matrix.getEulerYPR(yaw, pitch, roll);
		#endif

		switch (canSettings.IMU_axis.c_str()[0]) {
		case 'X':
			//ROS_INFO("Using X-Axis");
			IMU_angle = roll;
			break;
		case 'Z':
			//ROS_INFO("Using Z-Axis");
			IMU_angle = yaw;
			break;
		default:
			//ROS_INFO("Using Y-Axis");
			IMU_angle = pitch;
			break;
		}
		if (canSettings.IMU_inverted) {
			//ROS_INFO("Invert angle");
			IMU_angle = -IMU_angle;
		}
		if (canSettings.setoffset) {
			canSettings.IMU_angle_offset = IMU_angle;
			canSettings.setoffset = false;
		}
		IMU_angle -= canSettings.IMU_angle_offset;
		adoptPosition();
	}
}

void NiftiArmNode::calibrateCallback(const std_msgs::Empty::ConstPtr& msg) {
	canSettings.setoffset = true;
}

#endif /* CALLBACKS_H_ */
