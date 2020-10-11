#include <ros/ros.h>
#include <franka_modern_driver/franka_modern_driver_two_arms.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "franka_modern_driver_two_arms_node");
    ros::NodeHandle node_handle;

    std::string robot_ip1 = "192.168.1.50";
    std::string robot_ip2 = "192.168.1.51";
    franka_driver panda_(robot_ip1,robot_ip2);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    panda_.halt();

    return 0;
}
