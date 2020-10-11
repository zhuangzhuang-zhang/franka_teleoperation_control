#ifndef FRANKA_MODERN_DRIVER_H__
#define FRANKA_MODERN_DRIVER_H__

#include "franka_common.h"

class franka_driver
{
public:
    franka_driver(std::string robot_ip,int mode);

    ~franka_driver(){delete p_robot_;}

    void halt();

private:
    int mode_;
    franka::Robot *p_robot_;
    franka::RobotState robot_state_;
    bool trajectory_control_active_;
    bool servoj_control_active_;
    bool readOnce_shutdown;
    std::vector<double> servoj_cmd_;

    ros::NodeHandle n_;
    ros::Publisher joint_states_pub_;
    ros::Publisher robot_states_pub_;
    ros::Subscriber servoj_sub_;
    ros::ServiceServer error_recovery_ser_;
    control_msgs::FollowJointTrajectoryResult result_;
    bool has_goal_;
    bool reach_goal_;

    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;

    std::thread* state_pub_thread_;
    std::thread* servoj_thread_;

    franka_msgs::Errors errorsToMessage(const franka::Errors& error);
    void setDefaultBehavior(franka::Robot *robot);
    void franka_readOnce();
    void state_pub_thread_func();
    void servojInterface(const franka_msgs::servoj::ConstPtr& msg);
    bool errorRecovery(franka_msgs::error_recovery::Request &req,
                       franka_msgs::error_recovery::Response &res);
    void servoj_thread_func();
    void executeCB(actionlib::ServerGoalHandle<
                                  control_msgs::FollowJointTrajectoryAction> gh);
    bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);
    void trajThread(std::vector<double> timestamps,
            std::vector<std::vector<double> > positions,
            std::vector<std::vector<double> > velocities,
            std::vector<std::vector<double> > acceleration);
    void doTraj(std::vector<double> inp_timestamps,
                               std::vector<std::vector<double> > inp_positions,
                               std::vector<std::vector<double> > inp_velocities,
                               std::vector<std::vector<double> > inp_acceleration);
    std::vector<double> interp_cubic(double t, double T,
            std::vector<double> p0_pos, std::vector<double> p1_pos,
            std::vector<double> p0_vel, std::vector<double> p1_vel);
    std::vector<double> interp_fifth(double t, double T,
            std::vector<double> p0_pos, std::vector<double> p1_pos,
            std::vector<double> p0_vel, std::vector<double> p1_vel,
            std::vector<double> p0_acc, std::vector<double> p1_acc);
};

#endif
