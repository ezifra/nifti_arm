/*
 * msgthread.h
 *
 *  Created on: Feb 23, 2012
 *      Author: Daniel Robin Reuter
 */

#ifndef MSGTHREAD_H_
#define MSGTHREAD_H_
#include <qobject.h>
#include <qthread.h>
#include <ros/ros.h>
#include <nifti_arm_msgs/msg_height.h>
#include <nifti_arm_msgs/msg_angle.h>
#include <nifti_arm_msgs/msg_position.h>
#include <nifti_arm_msgs/msg_current.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
/*
typedef void (*cb_IMU_t)(const sensor_msgs::Imu::ConstPtr&);
typedef void (*cb_current_t)(const arm::msg_current::ConstPtr&);
typedef void (*cb_position_t)(const arm::msg_position::ConstPtr&);
*/
class msg_thread : public QThread {
	Q_OBJECT
public:
	msg_thread();
	virtual ~msg_thread();
	void publishHeight(int height);
	void publishAngle(float angle);
	void publishPan(float angle);
	void publishTilt(float angle);
	void publishCalibrate();
	void publishHome();
	void publishStop();
	void publishActivate();
	void publishManualActivate(bool status);
	void publishManualPosition(float shoulder, float elbow);
	ros::NodeHandle* getNH(){
		return n;
	}
protected:
	void run();
private:
	//ROS basics
    ros::NodeHandle *n;
    ros::Rate *rate;
    //Publisher
    ros::Publisher pubHeight;
    ros::Publisher pubAngle;
    ros::Publisher pubCalibrate;
    ros::Publisher pubHome;
    ros::Publisher pubStop;
    ros::Publisher pubPan;
    ros::Publisher pubTilt;
    ros::Publisher pubActivate;
    ros::Publisher pubManualActivate;
    ros::Publisher pubManualPosition;
};

#endif /* MSGTHREAD_H_ */
