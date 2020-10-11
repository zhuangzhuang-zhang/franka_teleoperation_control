#include <iostream>
#include <ros/ros.h>
#include <franka_msgs/servoj.h>

using namespace std;

int main(int argc,char **argv){

	ros::init(argc,argv,"test_node");
	ros::NodeHandle nh_;
    ros::Publisher pub = nh_.advertise<franka_msgs::servoj>("tool_servoj",10);
    ros::Rate loop_rate(100);
    while(ros::ok()){
		franka_msgs::servoj msg;
		msg.keepalive = true;
		msg.cmd_q.resize(4);
		msg.cmd_q[0] = 0;
		msg.cmd_q[1] = 0;
		msg.cmd_q[2] = 0;
		msg.cmd_q[3] = 0;
		pub.publish(msg);
		loop_rate.sleep();	
	}


	return 0;
}
