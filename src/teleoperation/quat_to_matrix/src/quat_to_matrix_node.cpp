#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace std;

int main(int argc,char **argv){
/*
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;
	tf::TransformListener listener;

  ros::Rate rate(2.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/temp", "/r_tool",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    Eigen::Quaterniond q(transform.getRotation().getW(),transform.getRotation().getX(),
                         transform.getRotation().getY(),transform.getRotation().getZ());


    Eigen::Matrix3d matrix = q.toRotationMatrix();

    cout<<matrix<<endl<<endl;


    double k2 = asin(-1*matrix(2,2));
    double k3 = asin(matrix(2,1)/cos(k2));
    double k1 = atan2(matrix(0,2)/(-1*cos(k2)),(matrix(1,2)/cos(k2)));

    double k2 = -1*asin(matrix(2,1));
    double k3 = asin(matrix(2,0)/cos(k2));
    double k1 = atan2(matrix(0,1)/(-1*cos(k2)),matrix(1,1)/cos(k2));


    cout<<k1<<"   "<<-1*k2<<"   "<<-1*k3<<endl;


    rate.sleep();
  }

*/

	Eigen::Matrix<double,3,3> m;
	m(0,0) = -1;	m(0,1) = 0;		m(0,2) = 0;
	m(1,0) = 0;		m(1,1) = 0;		m(1,2) = -1;
	m(2,0) = 0;		m(2,1) = -1;	m(2,2) = 0;
	Eigen::Quaterniond q(m);
	cout<<q.x()<<" "<<q.y()<<"  "<<q.z()<<" "<<q.w()<<endl;
	

	return 0;
}
