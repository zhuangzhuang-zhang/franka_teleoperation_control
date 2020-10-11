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

double pose_x_r =0;
double pose_y_r = 0;
double pose_z_r = 0;

double pose_x_r_fliter;
double pose_y_r_fliter;
double pose_z_r_fliter;

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



int main(int argc, char **argv)
{
    ros::init (argc, argv, "master_read_pose");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh;
    ros::Rate rate(FRE);


    bool filter = false;
    nh.getParam("/low_pass_fliter",filter);

    ros::Publisher pub_pose1=nh.advertise<robot_msgs::omega>("omega1/omega_map",100,true);
	ros::Publisher pub_pose2=nh.advertise<robot_msgs::omega>("omega2/omega_map",100,true);
    //ros::Publisher pub_pose_filter=nh.advertise<robot_msgs::omega>("/omega_map_filter",100,true);

    double rot_r = 0;
    double rot_p = 0;
    double rot_y =0;
    double gripper_angle=0;
    double rot_r_r = 0;
    double rot_p_r = 0;
    double rot_y_r =0;
    double gripper_angle_r=0;
    int done = 0;
    
    //get device count
    int device_count = dhdGetDeviceCount();
    if (device_count <= 0) {
        printf("error: %s\n", dhdErrorGetLastStr());
        ROS_ERROR("no device found");
        ros::shutdown();
        return 0;
    }
    if (device_count == 1) {
	ROS_ERROR("only one device found");
        ros::shutdown();
        return 0;
    }

    int id0 = dhdOpenID(0);
    if (id0 < 0) {
        printf("error: cannot open device 0(%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        ros::shutdown();
        return -1;
    }
    int id1 = dhdOpenID(1);
    if (id1 < 0) {
        printf("error: cannot open device 1(%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        ros::shutdown();
        return -1;
    }
    printf("%s device detected\n", dhdGetSystemName(id0));
    printf("%s device detected\n", dhdGetSystemName(id1));


    Eigen::Matrix<double,3,3> modify_matrix;
    modify_matrix(0,0) = -1;	        modify_matrix(0,1) = 0;		modify_matrix(0,2) = 0;
    modify_matrix(1,0) = 0;		modify_matrix(1,1) = -1;	modify_matrix(1,2) = 0;
    modify_matrix(2,0) = 0;		modify_matrix(2,1) = 0;		modify_matrix(2,2) = 1;

    // enable force
    dhdEnableForce(DHD_ON,id0);
    dhdEnableForce(DHD_ON,id1);
    while(ros::ok() && !done)
	{
        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, id0) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, id1) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        
        dhdGetGripperAngleRad(&gripper_angle,id1);//0.0001-0.4921
		if(gripper_angle<0)
			gripper_angle*=-1;
        dhdGetPosition(&pose_x, &pose_y, &pose_z,id1);
        dhdGetOrientationRad(&rot_r, &rot_p, &rot_y,id1);

        dhdGetGripperAngleRad(&gripper_angle_r,id0);//0.0001-0.4921
		if(gripper_angle_r<0)
			gripper_angle_r*=-1;
        dhdGetPosition(&pose_x_r, &pose_y_r, &pose_z_r,id0);
        dhdGetOrientationRad(&rot_r_r, &rot_p_r, &rot_y_r,id0);



        pose_x_fliter = pose_x;
        pose_y_fliter = pose_y;
        pose_z_fliter = pose_z;
        pose_x_r_fliter = pose_x_r;
        pose_y_r_fliter = pose_y_r;
        pose_z_r_fliter = pose_z_r;


	Eigen::Matrix<double,3,1> position_orign,position_modify;
	position_orign(0,0) = pose_x;	position_orign(1,0) = pose_y;	position_orign(2,0) = pose_z;
	position_modify = modify_matrix * position_orign;

	Eigen::Matrix<double,3,1> position_orign_r,position_modify_r;
	position_orign_r(0,0) = pose_x_r;	position_orign_r(1,0) = pose_y_r;	position_orign_r(2,0) = pose_z_r;
	position_modify_r = modify_matrix * position_orign_r;


	robot_msgs::omega msg1,msg2;
	msg1.data.resize(6);
	msg1.button.resize(1);
	msg2.data.resize(6);
	msg2.button.resize(1);
	msg1.data[0] = position_modify(0,0);	msg1.data[1] = position_modify(1,0);	msg1.data[2] = position_modify(2,0);
	msg1.data[3] = rot_r;	msg1.data[4] = rot_p;	msg1.data[5] = rot_y;
	msg2.data[0] = position_modify_r(0,0);	msg2.data[1] = position_modify_r(1,0);	msg2.data[2] = position_modify_r(2,0);
	msg2.data[3] = rot_r_r;	msg2.data[4] = rot_p_r;	msg2.data[5] = rot_y_r;
	msg1.button[0] = gripper_angle;
	msg2.button[0] = gripper_angle_r;


    pub_pose1.publish(msg1);
	pub_pose2.publish(msg2);




        bool isRate = rate.sleep();
        if(!isRate)
            std::cout<<"loop rate failed"<<std::endl;
    }

    dhdClose(id0);
    dhdClose(id1);
    std::cout<<"done.\n";
    ros::shutdown();

    return 0;

}
