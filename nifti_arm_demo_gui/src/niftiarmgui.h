#ifndef NIFTIARMGUI_H
#define NIFTIARMGUI_H

#include <QMainWindow>
#include "msgthread.h"
#include "RosCalls.h"

namespace Ui {
    class NiftiArmGUI;
}

class NiftiArmGUI : public QMainWindow {
    Q_OBJECT
public:
    NiftiArmGUI(QWidget *parent = 0);
    ~NiftiArmGUI();
	void displayCurrent(const nifti_arm_msgs::msg_current::ConstPtr& msg);
	void displayAngles(const nifti_arm_msgs::msg_position::ConstPtr& msg);
	void displayIMU(const sensor_msgs::Imu::ConstPtr& msg);
public slots:
	void clicked_b_height(void);
	void clicked_b_stop(void);
	void clicked_b_angle(void);
	void clicked_b_calibrate(void);
	void clicked_b_home(void);
	void clicked_b_activate(void);
	void clicked_b_refresh(void);
	void changed_s_height(int);
	void manual_changed(int);
	void pan_changed(int);
	void tilt_changed(int);
	void c_manual_changed(bool);


protected:
    void changeEvent(QEvent *e);

private:
    Ui::NiftiArmGUI *ui;
    msg_thread *rosthread;
    RosCalls rosparam;
    ros::NodeHandle* n;
    //Subscriber
    ros::Subscriber subIMU;
    ros::Subscriber subCurrent;
    ros::Subscriber subPosition;
};

#endif // NIFTIARMGUI_H
