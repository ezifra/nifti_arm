/*
 * dimensions.h
 *
 *  Created on: Mar 22, 2012
 *      Author: daniel
 */

#ifndef DIMENSIONS_T_H_
#define DIMENSIONS_T_H_

class dimensions_t {
public:
	double length_upper_arm; //This is the arm which is located UNDER the other one.
	double length_lower_arm;
	double mounting_height;
	double ptu_height;
	double ptu_height_camera;
	double max_position_error;
	double max_vel;
	double home_vel;
	double home_angle_shoulder;
	double home_angle_elbow;
	//dimensions_t& operator=(const dimensions_t &k);
	//bool operator==(const dimensions_t &k);
};

class canSettings_t {
public:
	int node_id_shoulder;int node_id_elbow;
	std::string CanDeviceFile;int CanBaudrate;
	std::string IMU_topic;
	std::string IMU_axis;
	bool IMU_inverted;
	double IMU_angle_offset;
	bool setoffset;
	//canSettings_t& operator=(const canSettings_t &k);
	//bool operator==(const canSettings_t &k);
};

/*dimensions_t& dimensions_t::operator=(const dimensions_t &k) {
 if (this != &k) {
 length_upper_arm = k.length_upper_arm;
 length_lower_arm = k.length_lower_arm;
 mounting_height = k.mounting_height;
 minimal_height = k.minimal_height;
 ptu_height = k.ptu_height;
 max_vel = k.max_vel;
 home_angle_shoulder = k.home_angle_shoulder;
 home_angle_elbow = k.home_angle_elbow;
 }
 return *this;
 }

 bool dimensions_t::operator==(const dimensions_t &k) {
 if (this != &k) {
 return length_upper_arm == k.length_upper_arm
 && length_lower_arm == k.length_lower_arm
 && mounting_height == k.mounting_height
 && ptu_height == k.ptu_height
 && minimal_height == k.minimal_height
 && home_angle_shoulder == k.home_angle_shoulder
 && home_angle_elbow == k.home_angle_elbow;
 } else {
 return true;
 }
 }

 canSettings_t& canSettings_t::operator=(const canSettings_t & k) {
 if (this != &k) {
 node_id_shoulder = k.node_id_shoulder;
 node_id_elbow = k.node_id_elbow;
 CanDeviceFile = k.CanDeviceFile;
 CanBaudrate = k.CanBaudrate;
 IMU_topic = k.IMU_topic;
 IMU_axis = k.IMU_axis;
 IMU_inverted = k.IMU_inverted;
 }
 return *this;
 }

 bool canSettings_t::operator==(const canSettings_t & k) {
 if (this != &k) {
 return node_id_shoulder == k.node_id_shoulder
 && node_id_elbow == k.node_id_elbow
 && CanDeviceFile == k.CanDeviceFile
 && CanBaudrate == k.CanBaudrate
 && IMU_topic == k.IMU_topic;
 } else {
 return true;
 }
 }*/

#endif /* DIMENSIONS_T_H_ */
