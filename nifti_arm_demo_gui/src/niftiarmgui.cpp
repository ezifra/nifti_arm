#include "niftiarmgui.h"
#include "ui_niftiarmgui.h"
#define PI 3.14159265

NiftiArmGUI::NiftiArmGUI(QWidget *parent) :
	QMainWindow(parent), ui(new Ui::NiftiArmGUI) {
	double ptu_height, ptu_height_camera, mounting_height, length_upper_arm,
			length_lower_arm, home_angle_shoulder, home_angle_elbow;
	ui->setupUi(this);
	QObject::connect(ui->b_height, SIGNAL( clicked() ), this,
			SLOT(clicked_b_height()));
	QObject::connect(ui->b_calibrate, SIGNAL( clicked() ), this,
			SLOT(clicked_b_calibrate()));
	QObject::connect(ui->b_home, SIGNAL( clicked() ), this,
			SLOT(clicked_b_home()));
	QObject::connect(ui->b_activate, SIGNAL( clicked() ), this,
			SLOT(clicked_b_activate()));
	QObject::connect(ui->b_angle, SIGNAL( clicked() ), this,
			SLOT(clicked_b_angle()));
	QObject::connect(ui->b_refresh, SIGNAL( clicked() ), this,
			SLOT( clicked_b_refresh() ));
	QObject::connect(ui->b_stop, SIGNAL( clicked() ), this,
			SLOT( clicked_b_stop() ));
	QObject::connect(ui->slide_height, SIGNAL( valueChanged(int) ), this,
			SLOT( changed_s_height(int) ));
	QObject::connect(ui->c_manual, SIGNAL( toggled(bool) ), this,
			SLOT( c_manual_changed(bool) ));
	QObject::connect(ui->s_motor_a, SIGNAL( valueChanged(int) ), this,
			SLOT( manual_changed(int) ));
	QObject::connect(ui->s_motor_b, SIGNAL( valueChanged(int) ), this,
			SLOT( manual_changed(int) ));
	QObject::connect(ui->s_pan, SIGNAL( valueChanged(int) ), this,
			SLOT( pan_changed(int) ));
	QObject::connect(ui->s_tilt, SIGNAL( valueChanged(int) ), this,
			SLOT( tilt_changed(int) ));
	rosthread = new msg_thread;
	rosthread->start();
	n = rosthread->getNH();
	n->getParam("/arm/dimensions/length_a", length_upper_arm);
	n->getParam("/arm/dimensions/length_b", length_lower_arm);
	n->getParam("/arm/dimensions/ptu_height_camera", ptu_height_camera);
	n->getParam("/arm/dimensions/home_angle_a", home_angle_shoulder);
	n->getParam("/arm/dimensions/home_angle_b", home_angle_elbow);
	n->getParam("/arm/dimensions/mounting_height", mounting_height);
	n->getParam("/arm/dimensions/ptu_height", ptu_height);
	ui->s_height->setMaximum(ptu_height + mounting_height + length_lower_arm
			+ length_upper_arm + ptu_height_camera);
	ui->slide_height->setMaximum(ptu_height + mounting_height
			+ length_lower_arm + length_upper_arm + ptu_height_camera);
	ui->slide_height->setMinimum(ptu_height + mounting_height
			+ ptu_height_camera + ((length_lower_arm * sin(-home_angle_elbow))
			+ (length_upper_arm * sin(-home_angle_shoulder))));
	subIMU = n->subscribe("/imu/data", 1, &NiftiArmGUI::displayIMU, this);
	subPosition = n->subscribe("/arm/position", 1, &NiftiArmGUI::displayAngles,
			this);
	subCurrent = n->subscribe("/arm/current", 1, &NiftiArmGUI::displayCurrent,
			this);
}

NiftiArmGUI::~NiftiArmGUI() {
	rosthread->quit();
	delete ui;
}

void NiftiArmGUI::manual_changed(int) {
	rosthread->publishManualPosition(ui->s_motor_a->value() / 18000.0 * PI,
			ui->s_motor_b->value() / 18000.0 * PI);
}

void NiftiArmGUI::pan_changed(int) {
	rosthread->publishPan(-ui->s_pan->value() / 18000.0 * PI);
}

void NiftiArmGUI::tilt_changed(int) {
	rosthread->publishTilt(ui->s_tilt->value() / 18000.0 * PI);
}

void NiftiArmGUI::c_manual_changed(bool s) {
	rosthread->publishManualActivate(s);
}

void NiftiArmGUI::clicked_b_refresh() {
	QStringList *parameterList;
	std::string tmp_str;
	QString tmp2;
	bool tmp_bool;
	double tmp_double;
	int tmp_int;
	QTableWidgetItem *item, *value, *type;
	parameterList = rosparam.allRosParameters();
	ui->tableWidget->setRowCount(parameterList->size() - 1);
	for (int i = 0; i < parameterList->size(); ++i) {
		item = new QTableWidgetItem;
		value = new QTableWidgetItem;
		type = new QTableWidgetItem;
		item->setText(parameterList->at(i));
		if (n->getParam(parameterList->at(i).toStdString(), tmp_str)) {
			tmp2 = tmp_str.c_str();
			type->setText("String");
		} else if (n->getParam(parameterList->at(i).toStdString(), tmp_bool)) {
			if (tmp_bool)
				tmp2 = "True";
			else
				tmp2 = "False";
			type->setText("Boolean");
		} else if (n->getParam(parameterList->at(i).toStdString(), tmp_double)) {
			tmp2.sprintf("%f", tmp_double);
			type->setText("Double");
		} else if (n->getParam(parameterList->at(i).toStdString(), tmp_int)) {
			tmp2.sprintf("%d", tmp_int);
			type->setText("Integer");
		}
		value->setText(tmp2);
		ui->tableWidget->setItem(i, 0, item);
		ui->tableWidget->setItem(i, 1, value);
		ui->tableWidget->setItem(i, 2, type);
	}
}

void NiftiArmGUI::changeEvent(QEvent *e) {
	QMainWindow::changeEvent(e);
	switch (e->type()) {
	case QEvent::LanguageChange:
		ui->retranslateUi(this);
		break;
	default:
		break;
	}
}

void NiftiArmGUI::clicked_b_height(void) {
	rosthread->publishHeight(ui->s_height->value());
}

void NiftiArmGUI::clicked_b_stop(void) {
	rosthread->publishStop();
}

void NiftiArmGUI::changed_s_height(int s) {
	rosthread->publishHeight(s);
}

void NiftiArmGUI::clicked_b_angle(void) {
	rosthread->publishAngle((ui->s_angle->value()) * PI / 180);
}

void NiftiArmGUI::clicked_b_calibrate(void) {
	rosthread->publishCalibrate();
}
void NiftiArmGUI::clicked_b_home(void) {
	rosthread->publishHome();
}
void NiftiArmGUI::clicked_b_activate(void) {
	rosthread->publishActivate();
}

void NiftiArmGUI::displayAngles(
		const nifti_arm_msgs::msg_position::ConstPtr& msg) {
	char* tmp = new char[30];
	QString tmp2;
	sprintf(tmp, "Motor A: %.1f degrees", msg->a * 180 / PI);
	tmp2 = tmp;
	ui->l_angle_motor_a->setText(tmp2);
	if (!ui->c_manual->isChecked())
		ui->s_motor_a->setValue(msg->a * 18000 / PI);
	sprintf(tmp, "Motor B: %.1f degrees", msg->b * 180 / PI);
	tmp2 = tmp;
	ui->l_angle_motor_b->setText(tmp2);
	if (!ui->c_manual->isChecked())
		ui->s_motor_b->setValue(msg->b * 18000 / PI);
}

void NiftiArmGUI::displayIMU(const sensor_msgs::Imu::ConstPtr& msg) {
	/*char* tmp = new char[30];
	 QString tmp2;
	 float angle;
	 tf::Quaternion q;
	 tf::quaternionMsgToTF(msg->orientation, q);
	 btMatrix3x3 matrix(q);
	 btScalar roll, pitch, yaw;
	 matrix.getEulerYPR(yaw, pitch, roll);
	 angle = roll;
	 sprintf(tmp, "IMU: %.1f degrees", angle * 180 / PI);
	 tmp2 = tmp;
	 ui->l_imu->setText(tmp2);*/
}

void NiftiArmGUI::displayCurrent(
		const nifti_arm_msgs::msg_current::ConstPtr& msg) {
	char* tmp = new char[30];
	QString tmp2;
	sprintf(tmp, "Motor A: %.3fA", msg->a);
	tmp2 = tmp;
	ui->l_current_a->setText(tmp2);
	sprintf(tmp, "Motor B: %.3fA", msg->b);
	tmp2 = tmp;
	ui->l_current_b->setText(tmp2);
	sprintf(tmp, "Sum: %.3fA", msg->a + msg->b);
	tmp2 = tmp;
	ui->l_current_sum->setText(tmp2);
}
