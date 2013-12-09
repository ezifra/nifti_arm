* Authors Daniel Reuter, Josef Boerding, Erik Zimmermann
* Licence BSD

1. Introduction/Overview
------------------------

The NIFTi arm is an extension "module" for the NIFTi-UGV, a [BlueBotics Absolem] (http://www.bluebotics.com/mobile-robotics/absolem/ "BlueBotics Absolem").
The arm has 4 degree of freedom. The two main actuators are lifting upper and lower joint to raise a hight up to ca. 1,20 m in addition to the robot hide. On top of the lower joint is a pan-till-unit (PTU) which can carry additional sensors and swivel those into the direction of choice, e.g. cameras. 
To support an easy replacement of sensors the PTU has 3 active USB 2.0 port available.

2. Installation
---------------

This software is based on [ROS Fuerte](http://wiki.ros.org/fuerte).
You need also the following packages (partly included here):
* [git] (http://git-scm.com)
* [nifti_arm_msgs] (https://github.com/NIFTi-Fraunhofer/nifti_arm/tree/master/nifti_arm_msgs)
* [dynamixel_msgs, dynamixel_controllers] (https://github.com/arebgun/dynamixel_motor) 
* [diagnostic_updater] (https://github.com/ros/diagnostics/tree/groovy-devel/diagnostic_updater) 
* [libcan] (https://github.com/NIFTi-Fraunhofer/nifti_arm/tree/master/libcan) 
* [libepos] (https://github.com/NIFTi-Fraunhofer/nifti_arm/tree/master/libepos)
* [tulibs] (https://github.com/NIFTi-Fraunhofer/nifti_arm/tree/master/tulibs)


2.1 Sources
-----------

You can get the sources by cloning the repository above:

*git clone https://github.com/NIFTi-Fraunhofer/nifti_arm*

The package *nifti_arm* and its dependencies is build with cmake. You can find a tutorial for using cmake [here](http://www.youtube.com/watch?v=CLvZTyji_Uw).

To build the *nifti_arm* sources, make sure you have the *nifti_arm* package and its dependencies set correctly in your ROS_PACKAGE_PATH environment variable.
Change into the checked out directory *nifti_arm*. Within each of the subdirectories (*libcan, libcpc, tulibs, libepos, nifti_arm_msgs, nifti_arm, nifti_arm_demo_gui*) execute the following commands:

*cmake .*  
*make*  
*sudo make install*



3. Usage
--------

To launch the *nifti_arm_demo_gui* just execute:

*roslaunch nifti_arm_demo_gui demo_gui.launch*

or if you have the arm installed:

*roslaunch nifti_arm_demo_gui demo_gui_with_arm.launch* 

For further documentation see [NIFTi_arm_demo_GUI_instructions.pdf](https://github.com/NIFTi-Fraunhofer/nifti_arm/blob/master/doc/NIFTi_arm_demo_GUI_instructions.pdf)




