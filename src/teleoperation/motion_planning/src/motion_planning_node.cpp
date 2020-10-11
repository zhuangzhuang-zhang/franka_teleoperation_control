#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <franka_msgs/servoj.h>
#include <trac_ik/trac_ik.hpp>  
#include <sensor_msgs/JointState.h>

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

using namespace std;

double joint_p[7];

std::vector<std::vector<double> > res;

void callback(sensor_msgs::JointStateConstPtr msg){
    for(int i=0;i<7;i++){
        joint_p[i] = msg->position[i];
    }
}

int main(int argc,char **argv){
    ros::init(argc, argv, "motion_planning_node");
    ros::NodeHandle nh_;
    ros::Publisher pub_ = nh_.advertise<franka_msgs::servoj>("franka2/franka/servoj",10);
    ros::Subscriber sub_joint_position = nh_.subscribe("franka2/joint_states", 100, callback);

    ros::Rate loop_rate(100);
    int count = 0;
    while(ros::ok()){
        count++;
        if(count == 300)
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }




    std::string chain_start = "panda2_link0";
    std::string chain_end = "panda2_tool";
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile("/home/master/franka_final_ws/src/franka_modern_driver/franka_description/robots/panda_arm_two.urdf", my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        exit(-1);
    }

    KDL::Chain my_chain;
    if(!my_tree.getChain(chain_start, chain_end, my_chain)){
        ROS_ERROR("Failed to convert to chain");
        exit(-1);
    }

    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(my_chain);
    unsigned int nj = my_chain.getNrOfJoints();
    if(nj!=7){
        ROS_ERROR("nj error!");
        exit(-1);
    }
    KDL::JntArray jointpositions = KDL::JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
      jointpositions(i)=joint_p[i];
    }

    KDL::Frame cartpos;

    int kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status<0){
        ROS_ERROR("fk error!");
        exit(-1);
    }












    double timeout = 0.005;
    std::string urdf_param = "/robot_description";
    double rcm_point[3]={0.762129,0.0588401,0.0927163};



    double rcmErrorThreshold = 0.02;
    TRAC_IK::TRAC_IK rcmSolver(chain_start, chain_end, rcm_point, rcmErrorThreshold, urdf_param, timeout, 1e-6);
    KDL::Chain rcm_chain;
    KDL::JntArray rcm_ll, rcm_ul;

    bool valid = rcmSolver.getKDLChain(rcm_chain);
    if (!valid) {
      ROS_ERROR("There was no valid KDL chain found in rcmSolver");
      exit(-1);
    }

    valid = rcmSolver.getKDLLimits(rcm_ll,rcm_ul);
    if (!valid) {
      ROS_ERROR("There were no valid KDL joint limits found in rcmSolver");
      exit(-1);
    }

    KDL::JntArray jointpositions_to_solve = KDL::JntArray(nj);
    KDL::JntArray joint_init = KDL::JntArray(nj);
    for(unsigned int i=0;i<nj;i++){
      joint_init(i)=joint_p[i];
    }

    double cir_y = cartpos.p[1];
	double r = 0.01;    
	double cir_z = cartpos.p[2]-r;
    
    int m = 7200;
    for(int i=0;i<m;i++){
        double d_y = cir_y-r*sin((double(i)/double(m))*6.28);
        double d_z = cir_z+r*cos((double(i)/double(m))*6.28);

        KDL::Frame d_cartpos;
        d_cartpos.p[0] = cartpos.p[0];
        d_cartpos.p[1] = d_y;
        d_cartpos.p[2] = d_z;

        int result = rcmSolver.CartToJntConstraint(joint_init,d_cartpos,jointpositions_to_solve);
        if(result>=0){
            std::vector<double> temp_p;
            for(int j=0;j<7;j++){
                temp_p.push_back(jointpositions_to_solve(j));
            }
            res.push_back(temp_p);
            for(unsigned int j=0;j<nj;j++){
              joint_init(j)=jointpositions_to_solve(j);
            }
        }else{
            std::cout<<"error!!!!!!!!!!!!"<<std::endl;
            exit(-1);
        }
    }

    std::cout<<"start move"<<std::endl;
    ros::Rate loop_rate1(400);
    int k=0;
    while(ros::ok())
    {
        double joint_distance = 0;
        for(int i=0;i<7;i++)
            joint_distance += pow((res[k][i]-joint_p[i]),2);
        joint_distance = sqrt(joint_distance);

        if(joint_distance<0.1){
               franka_msgs::servoj msg;
               msg.keepalive = 1;
               msg.cmd_q.resize(7);
               for(unsigned int i=0;i<7;i++)
                   msg.cmd_q[i] = res[k][i];

               pub_.publish(msg);
               k++;
			   std::cout<<"k:"<<k<<std::endl;
               if(k>=(m-3))
                   break;
        }else{
            std::cout<<"joint_distace: "<<joint_distance<<std::endl;
            exit(-1);
        }

        ros::spinOnce();
        loop_rate1.sleep();
    }

	std::cout<<"fininsh"<<std::endl;

    while(ros::ok())
    {
        franka_msgs::servoj msg;
        msg.keepalive = 0;
        pub_.publish(msg);
        loop_rate.sleep();

    }


	return 0;
}
