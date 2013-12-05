/*
 * NiftiArmNode.cpp
 *
 *  Created on: Mar 19, 2012
 *      Author: daniel
 */

#include "motorcontroller.h"

NiftiArmNode::NiftiArmNode() {
	operating = true;
	manual = false;
	homing = false;
	height = 0;
	nominal_angle = 0;
	IMU_angle = 0;
	n_status = new nifti_arm_status;
}

NiftiArmNode::~NiftiArmNode() {
	this->shutdown();
}

void NiftiArmNode::publishStatus() {
	double alpha, beta;
	alpha = epos_get_position(&MC_1) - dimensions.home_angle_shoulder;
	beta = epos_get_position(&MC_2) - dimensions.home_angle_elbow;
	n_status->position_elbow = alpha;
	n_status->position_shoulder = beta;
	nifti_arm_msgs::msg_position position_m;
	position_m.a = alpha;
	position_m.b = beta;
	pubPosition.publish(position_m);
	tf::Transform tf_shoulder, tf_elbow, tf_hand, tf_cam, tf_ptu, tf_ptu_joint;
	tf_shoulder.setRotation(tf::createQuaternionFromRPY(0, -alpha, 0));
	tf_shoulder.setOrigin(tf::Vector3(0, 0, dimensions.mounting_height / 1000));
	tf_cam.setRotation(tf::createQuaternionFromRPY(0, alpha, 0));
	tf_cam.setOrigin(tf::Vector3(dimensions.length_upper_arm / 1000, 0.05, 0));
	tf_elbow.setRotation(tf::createQuaternionFromRPY(0, -(PI - (beta)), 0));
	tf_elbow.setOrigin(tf::Vector3(dimensions.length_upper_arm / 1000, 0, 0));
	tf_hand.setRotation(tf::createQuaternionFromRPY(0, PI - beta + alpha, 0));
	tf_hand.setOrigin(tf::Vector3(dimensions.length_lower_arm / 1000, 0, 0));
	tf_ptu.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
	tf_ptu.setOrigin(tf::Vector3(0, 0, dimensions.ptu_height_camera / 1000));
	tf_ptu_joint.setRotation(tf::createQuaternionFromRPY(0, tilt_angle,
			pan_angle));
	tf_ptu_joint.setOrigin(tf::Vector3(0, 0, dimensions.ptu_height / 1000));

	tf_br->sendTransform(tf::StampedTransform(tf_ptu, ros::Time::now(),
			"ptu_joint", "ptu_mounting"));
	tf_br->sendTransform(tf::StampedTransform(tf_shoulder, ros::Time::now(),
			"base_arm", "shoulder"));
	tf_br->sendTransform(tf::StampedTransform(tf_elbow, ros::Time::now(),
			"shoulder", "elbow"));
	tf_br->sendTransform(tf::StampedTransform(tf_cam, ros::Time::now(),
			"shoulder", "elbow_mounting"));
	tf_br->sendTransform(tf::StampedTransform(tf_hand, ros::Time::now(),
			"elbow", "hand"));
	tf_br->sendTransform(tf::StampedTransform(tf_ptu_joint, ros::Time::now(),
			"hand", "ptu_joint"));

	nifti_arm_msgs::msg_current current_m;
	curr1 = elmo_get_current(&MC_1.dev);
	curr2 = elmo_get_current(&MC_2.dev);
	current_m.a = curr1;
	current_m.b = curr2;
	n_status->current_shoulder = curr1;
	n_status->current_elbow = curr2;
	pubCurrent.publish(current_m);
	if (manual) n_status->status = Manual;
	if (!operating) n_status->status = Deactivated;
	if (homing) n_status->status = Homing;
	updater->update();
}

int NiftiArmNode::adoptPosition(void) {
	if (!(!operating || manual)) {
		n_status->internal_height = height;
		n_status->height = (height + dimensions.mounting_height
				+ dimensions.ptu_height + dimensions.ptu_height_camera);
		epos_position_profile_t profileMC_shoulder, profileMC_elbow;
		ROS_DEBUG("Height: %fmm (%dmm)", (height + dimensions.mounting_height
				+ dimensions.ptu_height + dimensions.ptu_height_camera), height);
		double alpha, vel_alpha, vel_beta, beta;
		if (height > dimensions.length_upper_arm + dimensions.length_lower_arm) {
			ROS_ERROR(
					"Height is to big. Skipping the command; (Height: %fmm; Alpha: %f; Beta: %f)",
					(height + dimensions.mounting_height
							+ dimensions.ptu_height
							+ dimensions.ptu_height_camera), rad_to_deg(alpha),
					rad_to_deg(beta));
			updater->broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Height is to big. Skipping the command;");
			return 1;
		}
		calculateAngles(alpha, beta); //Calculating the angles
		alpha = alpha + dimensions.home_angle_shoulder; //Adding the home-angle
		beta = beta + dimensions.home_angle_elbow;
		if (height < ((dimensions.length_lower_arm * sin(
				-dimensions.home_angle_elbow)) + (dimensions.length_upper_arm
				* sin(-dimensions.home_angle_shoulder)))) {
			if (height != 0)
				ROS_INFO(
						"Height is to small. (Height: %dmm; Alpha: %f; Beta: %f) Setting height to Zero.",
						height, rad_to_deg(alpha), rad_to_deg(beta));
			updater->broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Height is to small. Setting height to Zero.");
			alpha = 0;
			beta = 0;
		}
		if (alpha > deg_to_rad(105)) {
			ROS_ERROR("Position not reachable. Skip.");
			updater->broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Height is not reachable. Skip.");
			return 1;
		}
		if (beta < 0) {
			beta = 0;
		}

		vel_alpha = (epos_get_position(&MC_1) - (alpha)) / (epos_get_position(
				&MC_2) - (beta));
		if (vel_alpha < 0)
			vel_alpha = -vel_alpha;
		if (isnan(vel_alpha)) {
			vel_alpha = dimensions.max_vel;
			vel_beta = dimensions.max_vel;
		} else if (vel_alpha > 1) {
			vel_beta = dimensions.max_vel / vel_alpha;
			if (isnan(vel_beta))
				vel_beta = dimensions.max_vel;
			vel_alpha = dimensions.max_vel;
		} else {
			vel_alpha = dimensions.max_vel * vel_alpha;
			vel_beta = dimensions.max_vel;
		}
		ROS_DEBUG("Vel: Alpha: %f, Beta %f", vel_alpha, vel_beta);

		//Starting Profile
		epos_position_profile_init(&profileMC_shoulder, alpha, vel_alpha,
				(float) (490000 / 360 * PI), (float) (490000 / 360 * PI),
				epos_profile_linear);
		profileMC_shoulder.relative = 0;
		epos_position_profile_init(&profileMC_elbow, beta, vel_beta,
				(float) (490000.0 / 360.0 * PI), (float) (490000.0 / 360 * PI),
				epos_profile_linear);
		profileMC_elbow.relative = 0;
		epos_position_profile_start(&MC_1, &profileMC_shoulder);
		epos_position_profile_start(&MC_2, &profileMC_elbow);
		/*if (height == 0) {
		 while ((!(epos_device_get_status(&MC_1.dev)
		 & ELMO_BITMASK_TARGET_REACHED))
		 || (!(epos_device_get_status(&MC_2.dev)
		 & ELMO_BITMASK_TARGET_REACHED))) {
		 prate->sleep();
		 //this->publishStatus();
		 }
		 epos_device_shutdown(&MC_1.dev);
		 epos_device_shutdown(&MC_2.dev);
		 }*/

		return 0; //Succeed
	}
	return 2;
}

void NiftiArmNode::currentControl() {
	if (((curr1 + curr2) > 2.5) || (curr1 > 1.2) || (curr2 > 1.2)) {
		this->floatMotors();
		ROS_ERROR("Current limit reached (%.2fA|%.2fA). Stopping Motors.",
				curr1, curr2);
		operating = false;
	}
}

void NiftiArmNode::position_error_Control() {
	double pos1, pos2;
	pos1 = elmo_get_position_error(&MC_1);
	pos2 = elmo_get_position_error(&MC_2);
	if (pos1 > dimensions.max_position_error) {
		epos_device_shutdown(&MC_1.dev);
		ROS_ERROR(
				"Position error limit reached (%.2f degrees|%.2f degrees). Stopping Motors.",
				rad_to_deg(pos1), rad_to_deg(pos2));
		if (operating) {
			epos_position_profile_t profileMC;
			epos_position_profile_init(&profileMC, 0, dimensions.max_vel,
					(float) (490000 / 360 * PI), (float) (490000 / 360 * PI),
					epos_profile_linear);
			profileMC.relative = 0;
			epos_position_profile_start(&MC_2, &profileMC);
		}
		operating = false;
	}
	if (pos2 > dimensions.max_position_error) {
		ROS_ERROR(
				"Position error limit reached (%.2f degrees|%.2f degrees). Stopping Motors.",
				rad_to_deg(pos1), rad_to_deg(pos2));
		epos_device_shutdown(&MC_2.dev);
		if (operating) {
			epos_position_profile_t profileMC;
			epos_position_profile_init(&profileMC, 0, dimensions.max_vel,
					(float) (490000 / 360 * PI), (float) (490000 / 360 * PI),
					epos_profile_linear);
			profileMC.relative = 0;
			epos_position_profile_start(&MC_1, &profileMC);
		}
		operating = false;
	}
}

void NiftiArmNode::floatMotors() {
	epos_device_shutdown(&MC_1.dev);
	epos_device_shutdown(&MC_2.dev);
}

int NiftiArmNode::initialize() {
	n_status->status = Initializing;
	getRosParameters();
	pubCurrent = n->advertise<nifti_arm_msgs::msg_current> ("/arm/current", 1);
	pubPosition = n->advertise<nifti_arm_msgs::msg_position> ("/arm/position",
			1);
	subOnlyHeight = n->subscribe("arm/height", 1,
			&NiftiArmNode::onlyHeightCallback, this);
	subAngle = n->subscribe("arm/angle", 1, &NiftiArmNode::angleCallback, this);
	subActivate = n->subscribe("arm/activate", 1,
			&NiftiArmNode::activateCallback, this);
	subStop = n->subscribe("arm/stop", 1, &NiftiArmNode::stopCallback, this);
	subHome = n->subscribe("arm/home", 1, &NiftiArmNode::homeCallback, this);
	subCalibrate = n->subscribe("arm/calibrate", 1,
			&NiftiArmNode::calibrateCallback, this);
	ROS_INFO("Listening on %s for IMU-Data", canSettings.IMU_topic.c_str());
	subIMU = n->subscribe(canSettings.IMU_topic, 1, &NiftiArmNode::IMUCallback,
			this);
	subPan = n->subscribe("/pan_controller/state", 1,
			&NiftiArmNode::PanCallback, this);
	subTilt = n->subscribe("/tilt_controller/state", 1,
			&NiftiArmNode::TiltCallback, this);
	subManualActivate = n->subscribe("arm/manualActivate", 1,
			&NiftiArmNode::manualActivateCallback, this);
	subManualPosition = n->subscribe("arm/manualPosition", 1,
			&NiftiArmNode::manualPositionCallback, this);
	config_t config;
	param_t param;
	//Initialize CAN-bus
	ROS_INFO("Opening Can-Interface (%s) with %d000 baud/s",
			canSettings.CanDeviceFile.c_str(), canSettings.CanBaudrate);
	config_init(&config);
	param_init_int(&param, "bitrate", canSettings.CanBaudrate);
	config_set_param(&config, &param);
	param_init_string(&param, "usb-dev", canSettings.CanDeviceFile.c_str());
	config_set_param(&config, &param);
	can_init(&can_dev, &config);
	config_destroy(&config);

	//Initialize motorcontroller
	if (initializeMotorControllerDevice(&MC_1, canSettings.node_id_shoulder,
			&can_dev)) {
		n->shutdown();
		return 1;
	}
	if (initializeMotorControllerDevice(&MC_2, canSettings.node_id_elbow,
			&can_dev)) {
		epos_close(&MC_1);
		epos_destroy(&MC_1);
		n->shutdown();
		return 1;
	}
	n_status->status = Ok;
	home();
	return 0;
}

void NiftiArmNode::home(void) {
	tStatus old_status;
	homing = true;
	old_status = n_status->status;
	n_status->status = Homing;
	int height_old;
	height_old = height;
	height = 200;
	adoptPosition();
	operating = false;
	while (((!(epos_device_get_status(&MC_1.dev) & ELMO_BITMASK_TARGET_REACHED))
			|| (!(epos_device_get_status(&MC_2.dev)
					& ELMO_BITMASK_TARGET_REACHED))) && n->ok()) {
		prate->sleep();
		this->publishStatus();
		this->position_error_Control();
		//this->publishStatus();
		//ros::spinOnce();
	}
	ROS_INFO("Start Homing shoulder");
	elmo_start_homing(&MC_1, dimensions.home_vel, (int)(490000.0 / 360 * PI),
			0, ELMO_HOMING_METHOD_NEG_HOME_SWITCH_RIGHT);
	elmo_wait_homed(&MC_1.dev, prate, this);
	ROS_INFO("Finished Homing shoulder");
	ROS_INFO("Start Homing elbow");
	elmo_start_homing(&MC_2, dimensions.home_vel, (int)(490000.0 / 360 * PI),
			0, ELMO_HOMING_METHOD_NEG_HOME_SWITCH_LEFT);
	elmo_wait_homed(&MC_2.dev, prate, this);
	ROS_INFO("Finished Homing elbow");
	operating = true;
	height = height_old;
	homing = false;
	n_status->status = old_status;
	adoptPosition();
}

void NiftiArmNode::calculateAngles(double &alpha, double &beta) {
	double a, b;
	a = dimensions.length_upper_arm;
	b = dimensions.length_lower_arm;

	if ((height * height) > ((a * a) - (b * b))) {
		alpha
				= asin(((height * height) + (a * a) - (b * b)) / (2 * height
						* a));
		beta = acos(cos(alpha) * a / b) + alpha;
	} else {
		ROS_INFO("Using approximation of the angles.");
		alpha = asin(height / a / 2);
		beta = 2 * alpha;
	}
	alpha = alpha + IMU_angle + nominal_angle; //Adding the angle of the IMU and the nominal angle
	if ((beta - alpha) < deg_to_rad(2.5)) {
		ROS_INFO("Using approximation of the angles.");
		alpha = asin(height / a / 2) + IMU_angle + nominal_angle;
		beta = 2 * asin(height / b / 2);
	}
	if (((beta - alpha) < deg_to_rad(2.5)) && (beta != 0 || alpha != 0)) {
		ROS_ERROR(
				"Position not reachable. Setting height to Zero. Alpha: %f, Beta %f",
				rad_to_deg(alpha), rad_to_deg(beta));
		alpha = 0;
		beta = 0;
	}
	/*if (alpha < 0) {
	 ROS_INFO("Angles too small; alpha %f; beta %f", rad_to_deg(alpha),
	 rad_to_deg(beta));
	 beta = 2 * PI - beta - (alpha - IMU_angle);
	 alpha = PI + alpha;
	 if (beta > (2 * PI - 2 * dimensions.home_angle_elbow))
	 beta = 2 * PI - 2 * dimensions.home_angle_elbow - 0.05;
	 }*/
}

void NiftiArmNode::shutdown(void) {
	int count = 0;
	height = 0;
	adoptPosition();
	while (((!(epos_device_get_status(&MC_1.dev) & ELMO_BITMASK_TARGET_REACHED))
			|| (!(epos_device_get_status(&MC_2.dev)
					& ELMO_BITMASK_TARGET_REACHED))) && (++count < 20
			* LOOP_RATE)) {
		prate->sleep();
	}
	if (count >= 20 * LOOP_RATE)
		ROS_INFO("Timout during shutdown.");
	epos_device_shutdown(&MC_1.dev);
	epos_device_shutdown(&MC_2.dev);
	epos_close(&MC_1);
	epos_destroy(&MC_1);
	epos_close(&MC_2);
	epos_destroy(&MC_2);
	n->shutdown();
}

void NiftiArmNode::getRosParameters(void) {
	if (!n->getParam("/arm/can/Interface", canSettings.CanDeviceFile)
			|| !n->getParam("/arm/can/bitrate", canSettings.CanBaudrate)
			|| !n->getParam("/arm/motor1/node_id", canSettings.node_id_shoulder)
			|| !n->getParam("/arm/motor2/node_id", canSettings.node_id_elbow)
			|| !n->getParam("/arm/constants/maximal_velocity",
					dimensions.max_vel) || !n->getParam(
			"/arm/constants/homing_velocity", dimensions.home_vel)
			|| !n->getParam("/arm/IMU/topic_name", canSettings.IMU_topic)
			|| !n->getParam("/arm/IMU/axis", canSettings.IMU_axis)
			|| !n->getParam("/arm/IMU/inverted", canSettings.IMU_inverted)) {
		ROS_ERROR("Couldn't get all parameters. Abort.");
		this->shutdown();
	} else {
		//ROS_INFO("Got all parameters.");
	}
	if (!n->getParam("/arm/dimensions/length_a", dimensions.length_upper_arm)
			|| !n->getParam("/arm/dimensions/length_b",
					dimensions.length_lower_arm) || !n->getParam(
			"/arm/dimensions/home_angle_a", dimensions.home_angle_shoulder)
			|| !n->getParam("/arm/dimensions/home_angle_b",
					dimensions.home_angle_elbow) || !n->getParam(
			"/arm/dimensions/mounting_height", dimensions.mounting_height)
			|| !n->getParam("/arm/dimensions/ptu_height", dimensions.ptu_height)
			|| !n->getParam("/arm/dimensions/ptu_height_camera",
					dimensions.ptu_height_camera)
			|| !n->getParam("/arm/dimensions/max_position_error",
					dimensions.max_position_error)) {
		ROS_ERROR("Couldn't get all dimensions. Using the default ones.");
		dimensions.length_upper_arm = 500.0;
		dimensions.length_lower_arm = 450.0;
		dimensions.home_angle_shoulder = 0;
		dimensions.home_angle_elbow = 0;
		dimensions.mounting_height = 0;
		dimensions.ptu_height = 0;
		dimensions.ptu_height_camera = 0;
		dimensions.max_vel = 0.3;
		dimensions.max_position_error = 0.087;
	}
	canSettings.IMU_angle_offset = 0;
	canSettings.setoffset = false;
}

#include "callbacks.h"
