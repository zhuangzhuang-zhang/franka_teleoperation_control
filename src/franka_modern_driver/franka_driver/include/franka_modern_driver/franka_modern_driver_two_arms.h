#ifndef FRANKA_MODERN_DRIVER_H__
#define FRANKA_MODERN_DRIVER_H__

#include "franka_common.h"

class franka_driver
{
public:
    franka_driver(std::string robot_ip1,std::string robot_ip2);

    ~franka_driver(){delete p_robot1_;delete p_robot2_;}

    void halt();

private:
    franka::Robot *p_robot1_;
    franka::Robot *p_robot2_;
    franka::RobotState robot_state1_;
    franka::RobotState robot_state2_;

    bool servoj_control_active1_;
    bool servoj_control_active2_;
    bool servoj_control_closed1_;
    bool servoj_control_closed2_;
    bool readOnce_shutdown;
    std::vector<double> servoj_cmd_;

    ros::NodeHandle n_;
    ros::Publisher joint_states_pub_;
    ros::Subscriber servoj_sub_;
    ros::ServiceServer error_recovery_ser_;

    std::thread* state_pub_thread_;
    std::thread* servoj_thread1_;
    std::thread* servoj_thread2_;

    void setDefaultBehavior(franka::Robot *robot);
    void franka_readOnce();
    void state_pub_thread_func();
    void servojInterface(const franka_msgs::servoj::ConstPtr& msg);
    bool errorRecovery(franka_msgs::error_recovery::Request &req,
                       franka_msgs::error_recovery::Response &res);
    void servoj_thread1_func();
    void servoj_thread2_func();
};

#endif
