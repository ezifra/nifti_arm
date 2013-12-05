/*
 * nifti_arm_status.h
 *
 *  Created on: Jun 11, 2012
 *      Author: daniel
 */

#ifndef NIFTI_ARM_STATUS_H_
#define NIFTI_ARM_STATUS_H_

enum tStatus {Ok, Manual, Homing, Error, Initializing, Unknown, Deactivated};

class nifti_arm_status {
public:
	nifti_arm_status();
	void getstatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
	float current_shoulder;
	float current_elbow;
	float position_shoulder;
	float position_elbow;
	float position_pan;
	float position_tilt;
	int height;
	int internal_height;
	tStatus status;
};

#include "nifti_arm_status.cpp"

#endif /* NIFTI_ARM_STATUS_H_ */
