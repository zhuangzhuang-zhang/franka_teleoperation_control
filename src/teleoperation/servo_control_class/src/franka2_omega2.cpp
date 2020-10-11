#include <iostream>
#include <servo_control_class/servo_control.h>

int main(int argc,char **argv)
{
    ros::init(argc, argv, "franka2_omega2_node");

    franka_teleoperation teleop("franka2","omega2");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    teleop.halt();
	ros::waitForShutdown();

	return 0;
}
