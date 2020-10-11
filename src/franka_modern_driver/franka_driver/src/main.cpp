#include <ros/ros.h>
#include <franka_modern_driver/franka_modern_driver.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "franka_modern_driver_node");
    ros::NodeHandle node_handle("~");

    std::string robot_ip;
	node_handle.param("robot_ip", robot_ip, std::string(""));
	if (robot_ip=="") {
	    ROS_FATAL("Missing robot_ip in launch file");
	    exit (-1);
	}

	franka_driver panda_(robot_ip,1);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    panda_.halt();

    return 0;
}
