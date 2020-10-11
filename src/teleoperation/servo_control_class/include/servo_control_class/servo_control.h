#ifndef SERVO_CONTROL_H__
#define SERVO_CONTROL_H__

#include <iostream>
#include <math.h>
#include <thread>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <fstream>

#include <linux/input.h> //for input_event
#include <fcntl.h> //for open()   
#include <unistd.h> //for read() & close()

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <robot_msgs/omega.h>
#include <robot_msgs/touch.h>
#include <franka_msgs/servoj.h>
#include <trac_ik/trac_ik.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>



class franka_teleoperation{
public:
    franka_teleoperation(std::string robot_name_, std::string master_name_);
    ~franka_teleoperation();

    void halt();

private:
    ros::NodeHandle nh_;
    unsigned int nj = 7;

    std::string robot_name;
    std::string master_name;

    bool sim=false;
    bool filter = false;
    bool control_activate_ = false;
	bool eye_hand_cooperation = false;
	bool rcm_enable = false;
    bool rcm_ready = false;
    bool useFootswitch = true;
    bool rcm_ik = true;
    std::string master_device = "omega";
    std::string chain_start, chain_end, urdf_param, urdf_file;
	std::string keyBoard;
    std::string footswitch;
	double rcm_point[3];
    double rcmErrorThreshold = 0.02;
	double camera_in_base1[7];//x,y,z,qx,qy,qz,qw
    
	ros::Publisher pub_franka_script;
    ros::Publisher pub_sim_servoj;
    ros::Publisher pub_tool;
    ros::Subscriber sub_joint_position;
    ros::Subscriber sub_touch_map;
    ros::Subscriber sub_omega_map;
    ros::Subscriber sub_tool_joints;

    std::thread* fk_thread_;
    std::thread* keyboard_thread_;
    std::thread* footswitch_thread_;
    std::thread* control_thread_;

    double joint_positions[7];
    double master1_pos[3];
    double master1_pos_zero[3];
    double master1_rpy[3];
    double master1_rpy_zero[3];
    int touch_button = 0;
    double omega_button = 0;

    int mode = 0;
    double direction1_x = 1;
    double direction1_y = 1;
    double direction1_z = 1;

    double direction1_rpy_r = -1;
    double direction1_rpy_p = -1;
    double direction1_rpy_y = 1;

    double scale_p_x=0.2;
    double scale_p_y=0.2;
    double scale_p_z=0.2;
    double scale_p_step = 0.02;
    double scale_p_max = 0.4;
    double scale_p_min = 0.02;

    double scale_r_x=0.04;
    double scale_r_y=0.04;
    double scale_r_z=0.04;
    double scale_r_step = 0.02;
    double scale_r_max = 0.5;
    double scale_r_min = 0.02;

    double tool_move = 0;
    double tool_step = 0.00003;

    Eigen::Matrix<double,4,4> camera_in_base1_matrix;//franka is base frame
    Eigen::Matrix<double,4,4> cam_to_screen;		 //camera is base frame

    Eigen::Matrix<double,4,4> cur_ee;

    Eigen::Matrix<double,4,4> slave1_zero;
    Eigen::Matrix<double,4,4> slave1_current_pose;
    Eigen::Matrix<double,4,4> slave1_zero_in_cam;

    void joint_position_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void tool_position_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void touch_map_callback(const robot_msgs::touch::ConstPtr& msg);
    void omega_map_callback(const robot_msgs::omega::ConstPtr& msg);

    void fk_func();
    void tool_fk_in_toolFrame();
    void keyboard_func();
    void footswitch_func();
    void control_func();

    //end_effector
    double tool_pos[3];
    Eigen::Matrix<double,3,3> curr_tool_orien_in_base;
    Eigen::Matrix<double,3,3> curr_tool_orien_in_tool;
    Eigen::Matrix<double,3,3> curr_tool_orien_in_cam;
    double tool1_max = 1.57;
    double tool1_min = -1.57;
    double tool2_max = 1.04;
    double tool2_min = -1.04;
    double tool3_max = 1.04;
    double tool3_min = -1.04;
    double gripper_max = 0.78;
    double gripper_min = 0;
};

#endif
