#include <QtGui/QApplication>
#include <ros/ros.h>
//#include "tf/tf.h"
//#include "tf/transform_broadcaster.h"
#include "niftiarmgui.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "arm_gui");
    NiftiArmGUI w;
    w.show();
    return a.exec();
}
