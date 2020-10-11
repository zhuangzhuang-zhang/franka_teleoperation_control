#include <franka_modern_driver/franka_modern_driver_two_arms.h>

franka_driver::franka_driver(std::string robot_ip1,std::string robot_ip2)
{
    p_robot1_ = new franka::Robot(robot_ip1);
    p_robot2_ = new franka::Robot(robot_ip2);
    servoj_control_active1_ = false;
	servoj_control_active2_ = false;
    servoj_control_closed1_ = true;
    servoj_control_closed2_ = true;
    readOnce_shutdown = false;
    servoj_cmd_.resize(14);
    setDefaultBehavior(p_robot1_);
    setDefaultBehavior(p_robot2_);

    state_pub_thread_ = new std::thread(boost::bind(&franka_driver::state_pub_thread_func,this));
    error_recovery_ser_ = n_.advertiseService("error_recovery", &franka_driver::errorRecovery, this);

    servoj_sub_ = n_.subscribe("franka/servoj", 1, &franka_driver::servojInterface, this);
    servoj_thread1_ = new std::thread(boost::bind(&franka_driver::servoj_thread1_func,this));
    servoj_thread2_ = new std::thread(boost::bind(&franka_driver::servoj_thread2_func,this));
}

void franka_driver::franka_readOnce()
{
    robot_state1_ = p_robot1_->readOnce();
    robot_state2_ = p_robot2_->readOnce();
}

void franka_driver::setDefaultBehavior(franka::Robot *robot)
{
    robot->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void franka_driver::state_pub_thread_func()
{
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states",1);
    sensor_msgs::JointState joint_states_msg;
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        if(readOnce_shutdown==false && !servoj_control_active1_ && !servoj_control_active2_ && servoj_control_closed1_ && servoj_control_closed2_)
        {
            franka_readOnce();
            readOnce_shutdown = false;
        }
        else
            readOnce_shutdown = true;

        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name.resize(14);
        joint_states_msg.position.resize(14);
        joint_states_msg.velocity.resize(14);
        joint_states_msg.effort.resize(14);

        joint_states_msg.name[0] = "l_panda_joint1";
        joint_states_msg.name[1] = "l_panda_joint2";
        joint_states_msg.name[2] = "l_panda_joint3";
        joint_states_msg.name[3] = "l_panda_joint4";
        joint_states_msg.name[4] = "l_panda_joint5";
        joint_states_msg.name[5] = "l_panda_joint6";
        joint_states_msg.name[6] = "l_panda_joint7";
        joint_states_msg.name[7] = "r_panda_joint1";
        joint_states_msg.name[8] = "r_panda_joint2";
        joint_states_msg.name[9] = "r_panda_joint3";
        joint_states_msg.name[10] = "r_panda_joint4";
        joint_states_msg.name[11] = "r_panda_joint5";
        joint_states_msg.name[12] = "r_panda_joint6";
        joint_states_msg.name[13] = "r_panda_joint7";

        for(unsigned int i=0;i<7;i++)
        {
            joint_states_msg.position[i] = robot_state1_.q[i];
            joint_states_msg.velocity[i] = robot_state1_.dq[i];
            joint_states_msg.effort[i] = robot_state1_.tau_J[i];
        }
        for(unsigned int i=0;i<7;i++)
        {
            joint_states_msg.position[i+7] = robot_state2_.q[i];
            joint_states_msg.velocity[i+7] = robot_state2_.dq[i];
            joint_states_msg.effort[i+7] = robot_state2_.tau_J[i];
        }

        joint_states_pub_.publish(joint_states_msg);
        loop_rate.sleep();
    }
}

void franka_driver::halt()
{
    state_pub_thread_->join();
    servoj_thread1_->join();
    servoj_thread2_->join();
}

void franka_driver::servojInterface(const franka_msgs::servoj::ConstPtr& msg)
{
    if(msg->keepalive==1 && msg->cmd_q.size()==14)
    {
        for(unsigned int i=0;i<14;i++)
            servoj_cmd_[i] = msg->cmd_q[i];
        servoj_control_active1_ = true;
		servoj_control_active2_ = true;
    }
    else{
        servoj_control_active1_ = false;
		servoj_control_active2_ = false;
	}
}

bool franka_driver::errorRecovery(franka_msgs::error_recovery::Request &req,
                   franka_msgs::error_recovery::Response &res)
{
    p_robot1_->automaticErrorRecovery();
    p_robot2_->automaticErrorRecovery();
    res.result = true;
    return true;
}

void franka_driver::servoj_thread1_func()
{
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        if(servoj_control_active1_ && readOnce_shutdown)
        {
            try
            {
				servoj_control_closed1_ = false;
                double time = 0.0;
                p_robot1_->control([this,&time](const franka::RobotState& robot_state,
                                                         franka::Duration period) -> franka::JointVelocities {
                    robot_state1_ = robot_state;
                    time += period.toSec();

                    double vel_cmd[7];
                    double k_scale = 13;
                    for(unsigned int i=0;i<7;i++)
                    {
                        vel_cmd[i] = (servoj_cmd_[i]-robot_state1_.q[i])*k_scale;
                    }

                    franka::JointVelocities velocities = {{vel_cmd[0],vel_cmd[1],vel_cmd[2],
                                                          vel_cmd[3],vel_cmd[4],vel_cmd[5],
                                                          vel_cmd[6]}};

                    if (!servoj_control_active1_)
                    {
                        franka::JointVelocities velocities_z = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
                        std::cout << std::endl << "Finished motion1" << std::endl;
                        return franka::MotionFinished(velocities_z);
                    }
                    return velocities;
                },franka::ControllerMode::kJointImpedance,true,100);
                std::cout << std::endl << "control1 shutdown" << std::endl;
                readOnce_shutdown = false;
				servoj_control_closed1_ = true;
            }
            catch (const franka::ControlException& e)
            {
                std::cout << e.what() << std::endl;
                std::cout << "Running error recovery..." << std::endl;
                p_robot1_->automaticErrorRecovery();
                readOnce_shutdown = false;
				servoj_control_closed1_ = true;
            }
        }
        else
            loop_rate.sleep();
    }
}


void franka_driver::servoj_thread2_func()
{
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        if(servoj_control_active2_ && readOnce_shutdown)
        {
            try
            {
				servoj_control_closed2_ = false;
                double time = 0.0;
                p_robot2_->control([this,&time](const franka::RobotState& robot_state,
                                                         franka::Duration period) -> franka::JointVelocities {
                    robot_state2_ = robot_state;
                    time += period.toSec();

                    double vel_cmd[7];
                    double k_scale = 13;
                    for(unsigned int i=0;i<7;i++)
                    {
                        vel_cmd[i] = (servoj_cmd_[i+7]-robot_state2_.q[i])*k_scale;
                    }

                    franka::JointVelocities velocities = {{vel_cmd[0],vel_cmd[1],vel_cmd[2],
                                                          vel_cmd[3],vel_cmd[4],vel_cmd[5],
                                                          vel_cmd[6]}};

                    if (!servoj_control_active2_)
                    {
                        franka::JointVelocities velocities_z = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
                        std::cout << std::endl << "Finished motion2" << std::endl;
                        return franka::MotionFinished(velocities_z);
                    }
                    return velocities;
                },franka::ControllerMode::kJointImpedance,true,100);
                std::cout << std::endl << "control2 shutdown" << std::endl;
                readOnce_shutdown = false;
				servoj_control_closed2_ = true;
            }
            catch (const franka::ControlException& e)
            {
                std::cout << e.what() << std::endl;
                std::cout << "Running error recovery..." << std::endl;
                p_robot2_->automaticErrorRecovery();
                readOnce_shutdown = false;
				servoj_control_closed2_ = true;
            }
        }
        else
            loop_rate.sleep();
    }
}
