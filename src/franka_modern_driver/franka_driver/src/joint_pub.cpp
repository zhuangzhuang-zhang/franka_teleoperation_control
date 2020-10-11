#include <iostream>
#include <thread>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

using namespace std;

double positions[14];
double velocities[14];
double efforts[14];

void Callback1(const sensor_msgs::JointState::ConstPtr &msg){
    for(int i=0;i<7;i++){
        positions[i] = msg->position[i];
        velocities[i] = msg->velocity[i];
        efforts[i] = msg->effort[i];
    }
}

void Callback2(const sensor_msgs::JointState::ConstPtr &msg){
    for(int i=0;i<7;i++){
        positions[i+7] = msg->position[i];
        velocities[i+7] = msg->velocity[i];
        efforts[i+7] = msg->effort[i];
    }
}

void joint_states_pub_thread()
{
    ros::NodeHandle n;
    ros::Publisher joint_states_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 100);

    ros::Rate loop_rate(200);

    sleep(1);
    while(ros::ok())
    {
        sensor_msgs::JointState joint_states_msg;

        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name.resize(14);
        joint_states_msg.position.resize(14);
        joint_states_msg.velocity.resize(14);
        joint_states_msg.effort.resize(14);

        joint_states_msg.name[0] = "franka1_panda_joint1";
        joint_states_msg.name[1] = "franka1_panda_joint2";
        joint_states_msg.name[2] = "franka1_panda_joint3";
        joint_states_msg.name[3] = "franka1_panda_joint4";
        joint_states_msg.name[4] = "franka1_panda_joint5";
        joint_states_msg.name[5] = "franka1_panda_joint6";
        joint_states_msg.name[6] = "franka1_panda_joint7";

        joint_states_msg.name[7] = "franka2_panda_joint1";
        joint_states_msg.name[8] = "franka2_panda_joint2";
        joint_states_msg.name[9] = "franka2_panda_joint3";
        joint_states_msg.name[10] = "franka2_panda_joint4";
        joint_states_msg.name[11] = "franka2_panda_joint5";
        joint_states_msg.name[12] = "franka2_panda_joint6";
        joint_states_msg.name[13] = "franka2_panda_joint7";

        for(unsigned int i=0;i<14;i++){
            joint_states_msg.position[i] = positions[i];
            joint_states_msg.velocity[i] = velocities[i];
            joint_states_msg.effort[i] = efforts[i];
        }

        joint_states_pub_.publish(joint_states_msg);
        loop_rate.sleep();
    }
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "joint_pub_node");
    ros::NodeHandle nh_;

    ros::Subscriber sub1 = nh_.subscribe("franka1/joint_states", 100, Callback1);
    ros::Subscriber sub2 = nh_.subscribe("franka2/joint_states", 100, Callback2);

    std::thread th1(joint_states_pub_thread);

    ros::spin();
    th1.join();

	return 0;
}
