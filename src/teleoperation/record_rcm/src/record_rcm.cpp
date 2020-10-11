#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
using namespace std;

int main(int argc,char **argv){

	ros::init(argc, argv, "record_rcm");

	ros::NodeHandle node;
	tf::TransformListener listener;

    ros::Rate rate(2.0);
	int count = 0;
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    while (node.ok()){
    try{
      listener.lookupTransform("/panda1_link0", "/panda1_tool",
                               ros::Time(0), transform1);
      listener.lookupTransform("/panda2_link0", "/panda2_tool",
                               ros::Time(0), transform2);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    count++;
	if(count >= 10)
		break;
    rate.sleep();
  }

	std::ofstream frcm;
    frcm.open("/home/master/franka_final_ws/src/rcm.txt",ios::trunc);
    

    if (!frcm.is_open())
    {
        ROS_ERROR("open file failed");
        exit(0);
    }

    frcm<<transform1.getOrigin().getX()<<std::endl;
	frcm<<transform1.getOrigin().getY()<<std::endl;
	frcm<<transform1.getOrigin().getZ()<<std::endl;
	frcm<<transform2.getOrigin().getX()<<std::endl;
	frcm<<transform2.getOrigin().getY()<<std::endl;
	frcm<<transform2.getOrigin().getZ()<<std::endl;

    frcm.close();


	return 0;
}
