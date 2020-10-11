#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <franka_msgs/servoj.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

extern "C" {
	#include "controlcan/controlcan.h"
}//Without extern "C", compiler cannot find lib functions   
#include "controlcan/can/can.h"
#include "controlcan/motor/motor.h"

using namespace std;

int motor1_id = 0x01;
int motor2_id = 0x02;
int motor3_id = 0x03;
int motor4_id = 0x04;
int motor5_id = 0x05;
int motor6_id = 0x06;
int motor7_id = 0x07;
int motor8_id = 0x08;

float ratio_motor1 = 0.667;
float ratio_couple = -0.6;

bool tool_enable = false;
double target_joint_position[8];
double current_joint_position[8];

double joint0_max = 90;
double joint0_min = -90;

double joint1_max = 60;
double joint1_min = -60;

double joint2_max = 60;
double joint2_min = -60;

double joint3_max = 45;
double joint3_min = 0;

void Callback1(const franka_msgs::servoj::ConstPtr& msg){
	if(msg->keepalive){
		
		double temp = 0;

	    temp = -1 * msg->cmd_q[0] / 3.1415926 * 180;
		if(temp>joint0_max)
			target_joint_position[0] = joint0_max;
		else if(temp<joint0_min)
			target_joint_position[0] = joint0_min;
		else
			target_joint_position[0] = temp;

		temp = msg->cmd_q[1] / 3.1415926 * 180;
		if(temp>joint1_max)
			target_joint_position[1] = joint1_max;
		else if(temp<joint1_min)
			target_joint_position[1] = joint1_min;
		else
			target_joint_position[1] = temp;

		temp = -1 * msg->cmd_q[2] / 3.1415926 * 180;
		if(temp>joint2_max)
			target_joint_position[2] = joint2_max;
		else if(temp<joint2_min)
			target_joint_position[2] = joint2_min;
		else
			target_joint_position[2] = temp;

		temp = msg->cmd_q[3] / 3.1415926 * 180;
		if(temp>joint3_max)
			target_joint_position[3] = joint3_max-5;
		else if(temp<joint3_min)
			target_joint_position[3] = joint3_min-5;
		else
			target_joint_position[3] = temp-5;
		
		tool_enable = true;
	}
	else
		tool_enable = false;
}

void Callback2(const franka_msgs::servoj::ConstPtr& msg){
	if(msg->keepalive){
		
		double temp = 0;

	    temp = -1 * msg->cmd_q[0] / 3.1415926 * 180;
		if(temp>joint0_max)
			target_joint_position[4] = joint0_max;
		else if(temp<joint0_min)
			target_joint_position[4] = joint0_min;
		else
			target_joint_position[4] = temp;

		temp = msg->cmd_q[1] / 3.1415926 * 180;
		if(temp>joint1_max)
			target_joint_position[5] = joint1_max;
		else if(temp<joint1_min)
			target_joint_position[5] = joint1_min;
		else
			target_joint_position[5] = temp;

		temp = -1 * msg->cmd_q[2] / 3.1415926 * 180;
		if(temp>joint2_max)
			target_joint_position[6] = joint2_max;
		else if(temp<joint2_min)
			target_joint_position[6] = joint2_min;
		else
			target_joint_position[6] = temp;

		temp = msg->cmd_q[3] / 3.1415926 * 180;
		if(temp>joint3_max)
			target_joint_position[7] = joint3_max-5;
		else if(temp<joint3_min)
			target_joint_position[7] = joint3_min-5;
		else
			target_joint_position[7] = temp-5;
		
		tool_enable = true;
	}
	else
		tool_enable = false;
}

int main(int argc,char **argv){
	ros::init(argc, argv, "tool_driver");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("franka1_tool_servoj", 100, Callback1);
	ros::Subscriber sub2 = n.subscribe("franka2_tool_servoj", 100, Callback2);
    ros::Publisher pub1 = n.advertise<sensor_msgs::JointState>("franka1_tool_states",100);
    ros::Publisher pub2 = n.advertise<sensor_msgs::JointState>("franka2_tool_states",100);

    for(int i=0;i<8;i++){
		target_joint_position[i]=0;
		current_joint_position[i]=0;
    }

	can_open();
	motor_init();
	motor_enable(Profile_Position_Mode, motor1_id);
	motor_enable(Profile_Position_Mode, motor2_id);
	motor_enable(Profile_Position_Mode, motor3_id);
	motor_enable(Profile_Position_Mode, motor4_id);

	motor_enable(Profile_Position_Mode, motor5_id);
	motor_enable(Profile_Position_Mode, motor6_id);
	motor_enable(Profile_Position_Mode, motor7_id);
	motor_enable(Profile_Position_Mode, motor8_id);

	ros::Rate loop_rate(50);
    Eigen::Matrix<float,4,1> curr_motor1(0.0,0.0,0.0,0.0);
    Eigen::Matrix<float,4,1> curr_motor2(0.0,0.0,0.0,0.0);
	while(ros::ok()){
		if(tool_enable){

				double joint_distance = 0;
				for(int i=0;i<8;i++){
					joint_distance += pow(target_joint_position[i]-current_joint_position[i],2);
				}
				joint_distance = sqrt(joint_distance);

                if(joint_distance<9){
                    Eigen::Matrix<float,4,1> joint_pos1(target_joint_position[0],target_joint_position[1],target_joint_position[2],target_joint_position[3]);
                    Eigen::Matrix<float,4,1> motor_pos1(0.0,0.0,0.0,0.0);

                    Eigen::Matrix<float,4,1> joint_pos2(target_joint_position[4],target_joint_position[5],target_joint_position[6],target_joint_position[7]);
                    Eigen::Matrix<float,4,1> motor_pos2(0.0,0.0,0.0,0.0);

                    Eigen::Matrix<float,4,4> transform_gripper_to_motor;
                    transform_gripper_to_motor << ratio_motor1,0,0,0,
                                                    0,-1,0,0,
                                                    0,ratio_couple,-1,+0.5,
                                                    0,ratio_couple,-1,-0.5;

                    motor_pos1 = transform_gripper_to_motor * joint_pos1;
                    motor_pos2 = transform_gripper_to_motor * joint_pos2;
                    //cout<<"set pos to: "<<motor_pos[0]<<", "<<motor_pos[1]<<", "<<motor_pos[2]<<", "<<motor_pos[3]<<endl;

                    double motor1_distance = 0;
                    double motor2_distance = 0;
                    for(int i=0;i<4;i++){
                        motor1_distance += pow(motor_pos1[i]-curr_motor1[i],2);
                        motor2_distance += pow(motor_pos2[i]-curr_motor2[i],2);
                    }
                    motor1_distance = sqrt(motor1_distance);
                    motor2_distance = sqrt(motor2_distance);

                    if(motor1_distance<5 && motor2_distance<5){
                        pos_set_abs_position(motor_pos1[0], motor1_id);
                        pos_set_abs_position(motor_pos1[1], motor2_id);
                        pos_set_abs_position(motor_pos1[2], motor3_id);
                        pos_set_abs_position(motor_pos1[3], motor4_id);

                        pos_set_abs_position(motor_pos2[0], motor5_id);
                        pos_set_abs_position(motor_pos2[1], motor6_id);
                        pos_set_abs_position(motor_pos2[2], motor7_id);
                        pos_set_abs_position(motor_pos2[3], motor8_id);

                        for(int i=0;i<4;i++){
                            curr_motor1[i] = motor_pos1[i];
                            curr_motor2[i] = motor_pos2[i];
                        }

                        for(int i=0;i<8;i++){
                            current_joint_position[i] = target_joint_position[i];
                        }
                    }else{
                        ROS_ERROR("motor_distance_too_large");
                    }
                }else{
                    ROS_ERROR("joint_distance_too_large");
                }
		}

        sensor_msgs::JointState tool_state1;
		tool_state1.header.stamp = ros::Time::now();
		tool_state1.position.resize(4);
		for(int i=0;i<4;i++){
			if(i==0 || i==2)
				tool_state1.position[i] = -1 * target_joint_position[i] / 180 * 3.1415926;
			else
				tool_state1.position[i] = target_joint_position[i] / 180 * 3.1415926;
		}
		pub1.publish(tool_state1);

        sensor_msgs::JointState tool_state2;
		tool_state2.header.stamp = ros::Time::now();
		tool_state2.position.resize(4);
		for(int i=0;i<4;i++){
            if(i==0 || i==2)
				tool_state2.position[i] = -1 * target_joint_position[i+4] / 180 * 3.1415926;
			else
 				tool_state2.position[i] = target_joint_position[i+4] / 180 * 3.1415926;
		}
		pub2.publish(tool_state2);

		ros::spinOnce();
		loop_rate.sleep();
	}

	pos_set_abs_position(0, motor1_id);
	pos_set_abs_position(0, motor2_id);
	pos_set_abs_position(0, motor3_id);
	pos_set_abs_position(0, motor4_id);

	pos_set_abs_position(0, motor5_id);
	pos_set_abs_position(0, motor6_id);
	pos_set_abs_position(0, motor7_id);
	pos_set_abs_position(0, motor8_id);

	sleep(2);
	pos_set_abs_position(0, motor1_id);
	pos_set_abs_position(0, motor2_id);
	pos_set_abs_position(0, motor3_id);
	pos_set_abs_position(0, motor4_id);

	pos_set_abs_position(0, motor5_id);
	pos_set_abs_position(0, motor6_id);
	pos_set_abs_position(0, motor7_id);
	pos_set_abs_position(0, motor8_id);
	sleep(2);
	
	return 0;
}
