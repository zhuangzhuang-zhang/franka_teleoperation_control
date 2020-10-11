#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcm_tests");
  ros::NodeHandle nh("~");
  
  ros::NodeHandle nh_;
  ros::Publisher joint_states_pub = nh_.advertise<sensor_msgs::JointState>("joint_states",10);
  ros::Publisher rcmErrorPub = nh_.advertise<std_msgs::Float64>("rcmError",10);
  ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);


  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  double eps = 1e-7;
  // double rcm[3]={0.58031, -0.28347, 0.18597};
  // double rcm[3]={0.56154, 0.15772, 0.32582};
  double rcm[3]={0.38071, -0.0053089, 0.29843};
  double rcmErrorThreshold = 0.001;
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, rcm, rcmErrorThreshold, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return -1;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return -1;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  // double init_p[7]={0.8170386000000001, -0.8768167199999999, -1.53035386, -2.3561232, -0.16340771999999992, 2.422667, 0.78520};
  // double init_p[7]={-0.6090124600000002, -0.96742464, 1.05519666, -2.4404794, 0.2526445599999998, 2.213, 0.7852};
  double init_p[7]={0.6686968399999995, -1.2022296, -0.99551228, -2.4404794, -0.19296018000000004, 1.57098775, 0.785};
  KDL::JntArray cur_p(chain.getNrOfJoints());
  for(int i=0;i<7;i++){
    cur_p(i) = init_p[i];
  }

  KDL::JntArray result;
  KDL::Frame end_effector_pose,last_pose;
  int rc;

  fk_solver.JntToCart(cur_p,end_effector_pose);


  //circle
  double r = 0.05;
  double cx = end_effector_pose.p(0);
  double cy = end_effector_pose.p(1)+r;

  ros::Rate loop_rate(300);
  double angle = 0;
  visualization_msgs::Marker line_strip,line_strip1;

  line_strip1.header.frame_id = "/r_panda_link0";
  line_strip1.header.stamp = ros::Time::now();
  line_strip1.ns = "lines";
  line_strip1.action = visualization_msgs::Marker::ADD;
  line_strip1.pose.orientation.w = 1.0;

  line_strip1.id = 2;
  line_strip1.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip1.scale.x = 0.003;
  line_strip1.color.b = 1.0;
  line_strip1.color.a = 1.0;

  for(int i=0;i<=100;i++){
    geometry_msgs::Point p;
    p.x = rcm[0]+rcmErrorThreshold*cos(6.28/100*i);
    p.y = rcm[1]+rcmErrorThreshold*sin(6.28/100*i);
    p.z = rcm[2];
    line_strip1.points.push_back(p);
  }
  while(ros::ok()){
    fk_solver.JntToCart(cur_p,last_pose);
    
    end_effector_pose.p(0) = cx+r*sin(angle);
    end_effector_pose.p(1) = cy-r*cos(angle); 
    

    rc=tracik_solver.CartToJntConstraint(cur_p,end_effector_pose,result);
    if(rc>=0){
      // std::cout<<"success: "<<angle<<std::endl;
    }else{
      std::cout<<"failed"<<std::endl;
      break;
    }

    angle += 0.00628/3;
    cur_p = result;



    //publish sim topic
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.position.resize(7);
    msg.name.resize(7);

    msg.name[0] = "panda_joint1";
    msg.name[1] = "panda_joint2";
    msg.name[2] = "panda_joint3";
    msg.name[3] = "panda_joint4";
    msg.name[4] = "panda_joint5";
    msg.name[5] = "panda_joint6";
    msg.name[6] = "panda_joint7";

    for(unsigned int i=0;i<7;i++)
        msg.position[i] = result(i);

    joint_states_pub.publish(msg);
    //--

    //calculate and pub rcmError
    Eigen::Matrix<double,3,1> z_;
    for(int i=0;i<3;i++)
      z_(i,0) = last_pose.M(i,2);
    Eigen::Matrix<double,3,1> end_p,rcm_p;
    for(int i=0;i<3;i++){
      end_p(i,0) = last_pose.p(i);
      rcm_p(i,0) = rcm[i];
    }
    double t = ( z_(0,0)*(rcm_p(0,0)-end_p(0,0))
      + z_(1,0)*(rcm_p(1,0)-end_p(1,0)) 
      + z_(2,0)*(rcm_p(2,0)-end_p(2,0)) ) / (z_(0,0)*z_(0,0)+z_(1,0)*z_(1,0)+z_(2,0)*z_(2,0));
    Eigen::Matrix<double,3,1> nearest_p;
    for(int i=0;i<3;i++)
      nearest_p(i,0) = end_p(i,0)+z_(i,0)*t;
    double rcm_error = (nearest_p(0,0)-rcm_p(0,0))*(nearest_p(0,0)-rcm_p(0,0))
    + (nearest_p(1,0)-rcm_p(1,0))*(nearest_p(1,0)-rcm_p(1,0))
    + (nearest_p(2,0)-rcm_p(2,0))*(nearest_p(2,0)-rcm_p(2,0));
    rcm_error = sqrt(rcm_error);
    std_msgs::Float64 err_msg;
    err_msg.data = rcm_error;
    rcmErrorPub.publish(err_msg);
    //--

    //pub marker
    
    line_strip.header.frame_id = "/r_panda_link0";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.01;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = end_effector_pose.p(0);
    p.y = end_effector_pose.p(1);
    p.z = end_effector_pose.p(2);
    if(line_strip.points.size()<=3000)
      line_strip.points.push_back(p);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_strip1);
    //--



    loop_rate.sleep();
  }


  return 0;
}
