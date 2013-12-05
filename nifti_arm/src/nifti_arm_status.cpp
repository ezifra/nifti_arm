/*
 * nifti_arm_status.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: daniel
 */

nifti_arm_status::nifti_arm_status() {
	current_elbow = 0;
	current_shoulder = 0;
	position_elbow = 0;
	position_shoulder = 0;
	internal_height = 0;
	height = 0;
	status = Unknown;
}

void nifti_arm_status::getstatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
	stat.add("Current (shoulder)", current_shoulder);
	stat.add("Current (elbow)", current_elbow);
	stat.add("Position (shoulder)", position_shoulder);
	stat.add("Position (elbow)", position_elbow);
	stat.add("Position (pan)", position_pan);
	stat.add("Position (tilt)", position_tilt);
	stat.add("Desired height", height);
	stat.add("Internal height", internal_height);
	switch(status)
	{
	 case Ok: stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything seems Ok.");
		 break;
	 case Initializing: stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Device initializes.");
		 break;
	 case Homing: stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Device is in homing mode.");
		 break;
	 case Error: stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Device is in Error mode.");
		 break;
	 case Manual: stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Device is in manual mode. Will not react on height commands.");
		 break;
	 case Deactivated: stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Device is deactivated. Need manual reactivation.");
		 break;
	 default: stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Unknown Status");
	}

}
