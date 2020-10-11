#include <ros/ros.h>
#include <franka_modern_driver/franka_modern_driver_v2.h>

int main(int argc, char** argv)
{
    std::string robot_ip = "192.168.1.50";
    std::string robot_name = "franka1";

    std::string node_name = robot_name + "franka_modern_driver_node_v2";
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;

	franka_driver panda_(robot_ip,robot_name,1);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    panda_.halt();

    return 0;
}
