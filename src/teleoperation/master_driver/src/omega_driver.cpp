#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <chrono>
#include "dhdc.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <ros/ros.h>
#include <robot_msgs/omega.h>

// some platforms do not define M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FRE 300

//#define SAVE_DATA_

double pose_x =0;
double pose_y = 0;
double pose_z = 0;

double pose_x_fliter;
double pose_y_fliter;
double pose_z_fliter;

#define NZEROS 2
#define NPOLES 2
#define GAIN   1.058546241e+03

static double x_xv[NZEROS+1], x_yv[NPOLES+1];
static double y_xv[NZEROS+1], y_yv[NPOLES+1];
static double z_xv[NZEROS+1], z_yv[NPOLES+1];

double filterloop_x(double input_)
{
    x_xv[0] = x_xv[1];
    x_xv[1] = x_xv[2];
    x_xv[2] = input_/ GAIN;
    x_yv[0] = x_yv[1];
    x_yv[1] = x_yv[2];
    x_yv[2] = (x_xv[0] + x_xv[2]) + 2 * x_xv[1] + ( -0.9149758348 * x_yv[0]) + (1.9111970674 * x_yv[1]);
    return x_yv[2];
}

double filterloop_y(double input_)
{
    y_xv[0] = y_xv[1];
    y_xv[1] = y_xv[2];
    y_xv[2] = input_/ GAIN;
    y_yv[0] = y_yv[1];
    y_yv[1] = y_yv[2];
    y_yv[2] = (y_xv[0] + y_xv[2]) + 2 * y_xv[1] + ( -0.9149758348 * y_yv[0]) + (1.9111970674 * y_yv[1]);
    return y_yv[2];
}

double filterloop_z(double input_)
{
    z_xv[0] = z_xv[1];
    z_xv[1] = z_xv[2];
    z_xv[2] = input_/ GAIN;
    z_yv[0] = z_yv[1];
    z_yv[1] = z_yv[2];
    z_yv[2] = (z_xv[0] + z_xv[2]) + 2 * z_xv[1] + ( -0.9149758348 * z_yv[0]) + (1.9111970674 * z_yv[1]);
    return z_yv[2];
}


void timerCallback(const ros::TimerEvent& e)
{
    pose_x_fliter = filterloop_x(pose_x);
    pose_y_fliter = filterloop_x(pose_y);
    pose_z_fliter = filterloop_x(pose_z);
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "master_read_pose");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh;
    ros::Rate rate(FRE);


    //ros::Timer timer = nh.createTimer(ros::Duration(0.00333), timerCallback);

    bool filter = false;
    //nh.getParam("/low_pass_fliter",filter);

    ros::Publisher pub_pose=nh.advertise<robot_msgs::omega>("omega2/omega_map",100,true);
    ros::Publisher pub_pose_filter=nh.advertise<robot_msgs::omega>("omega2/omega_map_filter",100,true);

    double rot_r = 0;
    double rot_p = 0;
    double rot_y =0;
    double gripper_angle=0;
    int done = 0;
    
    //get device count
    if (dhdGetDeviceCount() <= 0) {
        printf("error: %s\n", dhdErrorGetLastStr());
        ros::shutdown();
        return 0;
    }
    if (dhdOpen() < 0) {
        printf("error: cannot open device (%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        ros::shutdown();
        return -1;
    }
    printf("%s device detected\n", dhdGetSystemName());

#ifdef SAVE_DATA_
    std::ofstream x_fout,x_fliter_fout;
    x_fout.open("/home/smj/x.txt");
    x_fliter_fout.open("/home/smj/x_fliter.txt");

    if (!x_fout.is_open() || !x_fliter_fout.is_open())
    {
        ROS_ERROR("open file failed");
        exit(0);
    }
#endif

    Eigen::Matrix<double,3,3> modify_matrix;
    modify_matrix(0,0) = -1;	modify_matrix(0,1) = 0;		modify_matrix(0,2) = 0;
    modify_matrix(1,0) = 0;		modify_matrix(1,1) = -1;	modify_matrix(1,2) = 0;
    modify_matrix(2,0) = 0;		modify_matrix(2,1) = 0;		modify_matrix(2,2) = 1;

    // enable force
    dhdEnableForce(DHD_ON);
    while(ros::ok() && !done)
	{
        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        
        dhdGetGripperAngleRad(&gripper_angle);//0.0001-0.4921
        if(gripper_angle<0){
			gripper_angle = gripper_angle * -1.0;
        }  
        //get pose
        dhdGetPosition(&pose_x, &pose_y, &pose_z);
        dhdGetOrientationRad(&rot_r, &rot_p, &rot_y);


        if(filter)
        {
            pose_x_fliter = filterloop_x(pose_x);
            pose_y_fliter = filterloop_y(pose_y);
            pose_z_fliter = filterloop_z(pose_z);
        }
        else
        {
            pose_x_fliter = pose_x;
            pose_y_fliter = pose_y;
            pose_z_fliter = pose_z;
        }

		Eigen::Matrix<double,3,1> position_orign,position_modify;
		position_orign(0,0) = pose_x;	position_orign(1,0) = pose_y;	position_orign(2,0) = pose_z;
		position_modify = modify_matrix * position_orign;

		robot_msgs::omega msg;
		msg.data.resize(6);
		msg.button.resize(1);
		msg.data[0] = position_modify(0,0);	msg.data[1] = position_modify(1,0);	msg.data[2] = position_modify(2,0);
		msg.data[3] = rot_r;	msg.data[4] = rot_p;	msg.data[5] = rot_y;
        msg.button[0] = gripper_angle;
        pub_pose.publish(msg);

/*
        Eigen::Matrix<double,3,1> position_orign_fliter,position_modify_fliter;
        position_orign_fliter(0,0) = pose_x_fliter;	position_orign_fliter(1,0) = pose_y_fliter;	position_orign_fliter(2,0) = pose_z_fliter;
        position_modify_fliter = modify_matrix * position_orign_fliter;

        robot_msgs::omega msg_fliter;
        msg_fliter.data.resize(6);
		msg_fliter.button.resize(1);
        msg_fliter.data[0] = position_modify_fliter(0,0);	msg_fliter.data[1] = position_modify_fliter(1,0);	msg_fliter.data[2] = position_modify_fliter(2,0);
        msg_fliter.data[3] = rot_r;	msg_fliter.data[4] = rot_p;	msg_fliter.data[5] = rot_y;
        msg_fliter.button[0] = gripper_angle;

        pub_pose_filter.publish(msg_fliter);
*/

#ifdef SAVE_DATA_
        x_fout<<position_modify(0,0)<<std::endl;
        x_fliter_fout<<position_modify_fliter(0,0)<<std::endl;
#endif

        bool isRate = rate.sleep();
        if(!isRate)
            std::cout<<"loop rate failed"<<std::endl;
    }

#ifdef SAVE_DATA_
    x_fout.close();
    x_fliter_fout.close();
#endif

    dhdClose();
    std::cout<<"done.\n";
    ros::shutdown();

    return 0;

}
