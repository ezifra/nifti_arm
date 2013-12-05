#include "global.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm");
	ros::NodeHandle n;
	NiftiArmNode arm;
	diagnostic_updater::Updater updater;
	updater.setHardwareID("none");
	updater.add("arm status", arm.n_status, &nifti_arm_status::getstatus);



	ros::Rate r(LOOP_RATE);
	prate = &r;
	arm.setNodehandle(&n);
	arm.setUpdater(&updater);
	arm.initialize();
	while (ros::ok()) {
		arm.publishStatus();
		arm.position_error_Control();
		//arm.getRosParameters();
		r.sleep();
		ros::spinOnce();
	}

	return 0;
}
