#include <franka_modern_driver/franka_modern_driver_v2.h>

franka_driver::franka_driver(std::string robot_ip,std::string robot_name,int mode)
    : as_(n_,robot_name+"/follow_joint_trajectory",boost::bind(&franka_driver::executeCB, this, _1), false){
    robot_name_ = robot_name;
    mode_ = mode;
    p_robot_ = new franka::Robot(robot_ip);
    trajectory_control_active_ = false;
    servoj_control_active_ = false;
    readOnce_shutdown = false;
    has_goal_ = false;
    reach_goal_ = false;
    servoj_cmd_.resize(7);
    setDefaultBehavior(p_robot_);

    state_pub_thread_ = new std::thread(boost::bind(&franka_driver::state_pub_thread_func,this));
    error_recovery_ser_ = n_.advertiseService(robot_name_+"/error_recovery", &franka_driver::errorRecovery, this);

    if(mode_==1){
        servoj_sub_ = n_.subscribe(robot_name_+"/franka/servoj", 1, &franka_driver::servojInterface, this);
        servoj_thread_ = new std::thread(boost::bind(&franka_driver::servoj_thread_func,this));
    }
    else if(mode_ == 2){
        as_.start();
    }
}

void franka_driver::franka_readOnce(){
    robot_state_ = p_robot_->readOnce();
}

void franka_driver::setDefaultBehavior(franka::Robot *robot){
    robot->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

franka_msgs::Errors franka_driver::errorsToMessage(const franka::Errors& error) {
  franka_msgs::Errors message;
  message.joint_position_limits_violation =
      static_cast<decltype(message.joint_position_limits_violation)>(
          error.joint_position_limits_violation);
  message.cartesian_position_limits_violation =
      static_cast<decltype(message.cartesian_position_limits_violation)>(
          error.cartesian_position_limits_violation);
  message.self_collision_avoidance_violation =
      static_cast<decltype(message.self_collision_avoidance_violation)>(
          error.self_collision_avoidance_violation);
  message.joint_velocity_violation =
      static_cast<decltype(message.joint_velocity_violation)>(error.joint_velocity_violation);
  message.cartesian_velocity_violation =
      static_cast<decltype(message.cartesian_velocity_violation)>(
          error.cartesian_velocity_violation);
  message.force_control_safety_violation =
      static_cast<decltype(message.force_control_safety_violation)>(
          error.force_control_safety_violation);
  message.joint_reflex = static_cast<decltype(message.joint_reflex)>(error.joint_reflex);
  message.cartesian_reflex =
      static_cast<decltype(message.cartesian_reflex)>(error.cartesian_reflex);
  message.max_goal_pose_deviation_violation =
      static_cast<decltype(message.max_goal_pose_deviation_violation)>(
          error.max_goal_pose_deviation_violation);
  message.max_path_pose_deviation_violation =
      static_cast<decltype(message.max_path_pose_deviation_violation)>(
          error.max_path_pose_deviation_violation);
  message.cartesian_velocity_profile_safety_violation =
      static_cast<decltype(message.cartesian_velocity_profile_safety_violation)>(
          error.cartesian_velocity_profile_safety_violation);
  message.joint_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.joint_position_motion_generator_start_pose_invalid)>(
          error.joint_position_motion_generator_start_pose_invalid);
  message.joint_motion_generator_position_limits_violation =
      static_cast<decltype(message.joint_motion_generator_position_limits_violation)>(
          error.joint_motion_generator_position_limits_violation);
  message.joint_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.joint_motion_generator_velocity_limits_violation)>(
          error.joint_motion_generator_velocity_limits_violation);
  message.joint_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.joint_motion_generator_velocity_discontinuity)>(
          error.joint_motion_generator_velocity_discontinuity);
  message.joint_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.joint_motion_generator_acceleration_discontinuity)>(
          error.joint_motion_generator_acceleration_discontinuity);
  message.cartesian_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.cartesian_position_motion_generator_start_pose_invalid)>(
          error.cartesian_position_motion_generator_start_pose_invalid);
  message.cartesian_motion_generator_elbow_limit_violation =
      static_cast<decltype(message.cartesian_motion_generator_elbow_limit_violation)>(
          error.cartesian_motion_generator_elbow_limit_violation);
  message.cartesian_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_velocity_limits_violation)>(
          error.cartesian_motion_generator_velocity_limits_violation);
  message.cartesian_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_velocity_discontinuity)>(
          error.cartesian_motion_generator_velocity_discontinuity);
  message.cartesian_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_acceleration_discontinuity)>(
          error.cartesian_motion_generator_acceleration_discontinuity);
  message.cartesian_motion_generator_elbow_sign_inconsistent =
      static_cast<decltype(message.cartesian_motion_generator_elbow_sign_inconsistent)>(
          error.cartesian_motion_generator_elbow_sign_inconsistent);
  message.cartesian_motion_generator_start_elbow_invalid =
      static_cast<decltype(message.cartesian_motion_generator_start_elbow_invalid)>(
          error.cartesian_motion_generator_start_elbow_invalid);
  message.cartesian_motion_generator_joint_position_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_position_limits_violation)>(
          error.cartesian_motion_generator_joint_position_limits_violation);
  message.cartesian_motion_generator_joint_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_limits_violation)>(
          error.cartesian_motion_generator_joint_velocity_limits_violation);
  message.cartesian_motion_generator_joint_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_discontinuity)>(
          error.cartesian_motion_generator_joint_velocity_discontinuity);
  message.cartesian_motion_generator_joint_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_acceleration_discontinuity)>(
          error.cartesian_motion_generator_joint_acceleration_discontinuity);
  message.cartesian_position_motion_generator_invalid_frame =
      static_cast<decltype(message.cartesian_position_motion_generator_invalid_frame)>(
          error.cartesian_position_motion_generator_invalid_frame);
  message.force_controller_desired_force_tolerance_violation =
      static_cast<decltype(message.force_controller_desired_force_tolerance_violation)>(
          error.force_controller_desired_force_tolerance_violation);
  message.controller_torque_discontinuity =
      static_cast<decltype(message.controller_torque_discontinuity)>(
          error.controller_torque_discontinuity);
  message.start_elbow_sign_inconsistent =
      static_cast<decltype(message.start_elbow_sign_inconsistent)>(
          error.start_elbow_sign_inconsistent);
  message.communication_constraints_violation =
      static_cast<decltype(message.communication_constraints_violation)>(
          error.communication_constraints_violation);
  message.power_limit_violation =
      static_cast<decltype(message.power_limit_violation)>(error.power_limit_violation);
  message.joint_p2p_insufficient_torque_for_planning =
      static_cast<decltype(message.joint_p2p_insufficient_torque_for_planning)>(
          error.joint_p2p_insufficient_torque_for_planning);
  message.tau_j_range_violation =
      static_cast<decltype(message.tau_j_range_violation)>(error.tau_j_range_violation);
  message.instability_detected =
      static_cast<decltype(message.instability_detected)>(error.instability_detected);
  return message;
}

void franka_driver::state_pub_thread_func(){
    joint_states_pub_ = n_.advertise<sensor_msgs::JointState>(robot_name_+"/joint_states",1);
    robot_states_pub_ = n_.advertise<franka_msgs::FrankaState>(robot_name_+"/franka_state",1);
    sensor_msgs::JointState joint_states_msg;
    franka_msgs::FrankaState franka_state_msg;
    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if(!trajectory_control_active_ && readOnce_shutdown==false && !servoj_control_active_){
            franka_readOnce();
            readOnce_shutdown = false;
        }
        else
            readOnce_shutdown = true;

        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name.resize(7);
        joint_states_msg.position.resize(7);
        joint_states_msg.velocity.resize(7);
        joint_states_msg.effort.resize(7);

        joint_states_msg.name[0] = robot_name_+"_panda_joint1";
        joint_states_msg.name[1] = robot_name_+"_panda_joint2";
        joint_states_msg.name[2] = robot_name_+"_panda_joint3";
        joint_states_msg.name[3] = robot_name_+"_panda_joint4";
        joint_states_msg.name[4] = robot_name_+"_panda_joint5";
        joint_states_msg.name[5] = robot_name_+"_panda_joint6";
        joint_states_msg.name[6] = robot_name_+"_panda_joint7";

        for(unsigned int i=0;i<7;i++){
            joint_states_msg.position[i] = robot_state_.q[i];
            joint_states_msg.velocity[i] = robot_state_.dq[i];
            joint_states_msg.effort[i] = robot_state_.tau_J[i];
        }

        for (unsigned int i = 0; i < robot_state_.cartesian_collision.size(); i++) {
          franka_state_msg.cartesian_collision[i] = robot_state_.cartesian_collision[i];
          franka_state_msg.cartesian_contact[i] = robot_state_.cartesian_contact[i];
          franka_state_msg.K_F_ext_hat_K[i] = robot_state_.K_F_ext_hat_K[i];
          franka_state_msg.O_F_ext_hat_K[i] = robot_state_.O_F_ext_hat_K[i];
        }

        for (unsigned int i = 0; i < robot_state_.q.size(); i++) {
          franka_state_msg.q[i] = robot_state_.q[i];
          franka_state_msg.q_d[i] = robot_state_.q_d[i];
          franka_state_msg.dq[i] = robot_state_.dq[i];
          franka_state_msg.dq_d[i] = robot_state_.dq_d[i];
          franka_state_msg.tau_J[i] = robot_state_.tau_J[i];
          franka_state_msg.dtau_J[i] = robot_state_.dtau_J[i];
          franka_state_msg.tau_J_d[i] = robot_state_.tau_J_d[i];
          franka_state_msg.theta[i] = robot_state_.theta[i];
          franka_state_msg.dtheta[i] = robot_state_.dtheta[i];
          franka_state_msg.joint_collision[i] = robot_state_.joint_collision[i];
          franka_state_msg.joint_contact[i] = robot_state_.joint_contact[i];
          franka_state_msg.tau_ext_hat_filtered[i] = robot_state_.tau_ext_hat_filtered[i];
        }

        for (unsigned int i = 0; i < robot_state_.elbow.size(); i++) {
          franka_state_msg.elbow[i] = robot_state_.elbow[i];
        }

        for (unsigned int i = 0; i < robot_state_.elbow_d.size(); i++) {
          franka_state_msg.elbow_d[i] = robot_state_.elbow_d[i];
        }

        for (unsigned int i = 0; i < robot_state_.O_T_EE.size(); i++) {
          franka_state_msg.O_T_EE[i] = robot_state_.O_T_EE[i];
          franka_state_msg.F_T_EE[i] = robot_state_.F_T_EE[i];
          franka_state_msg.EE_T_K[i] = robot_state_.EE_T_K[i];
          franka_state_msg.O_T_EE_d[i] = robot_state_.O_T_EE_d[i];
        }
        franka_state_msg.m_ee = robot_state_.m_ee;
        franka_state_msg.m_load = robot_state_.m_load;
        franka_state_msg.m_total = robot_state_.m_total;

        for (unsigned int i = 0; i < robot_state_.I_load.size(); i++) {
          franka_state_msg.I_ee[i] = robot_state_.I_ee[i];
          franka_state_msg.I_load[i] = robot_state_.I_load[i];
          franka_state_msg.I_total[i] = robot_state_.I_total[i];
        }

        for (unsigned int i = 0; i < robot_state_.F_x_Cload.size(); i++) {
          franka_state_msg.F_x_Cee[i] = robot_state_.F_x_Cee[i];
          franka_state_msg.F_x_Cload[i] = robot_state_.F_x_Cload[i];
          franka_state_msg.F_x_Ctotal[i] = robot_state_.F_x_Ctotal[i];
        }

        franka_state_msg.time = robot_state_.time.toSec();
        franka_state_msg.current_errors = errorsToMessage(robot_state_.current_errors);
        franka_state_msg.last_motion_errors = errorsToMessage(robot_state_.last_motion_errors);

        switch (robot_state_.robot_mode) {
          case franka::RobotMode::kOther:
            franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_OTHER;
            break;

          case franka::RobotMode::kIdle:
            franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_IDLE;
            break;

          case franka::RobotMode::kMove:
            franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_MOVE;
            break;

          case franka::RobotMode::kGuiding:
            franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_GUIDING;
            break;

          case franka::RobotMode::kReflex:
            franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_REFLEX;
            break;

          case franka::RobotMode::kUserStopped:
            franka_state_msg.robot_mode =
                franka_msgs::FrankaState::ROBOT_MODE_USER_STOPPED;
            break;

          case franka::RobotMode::kAutomaticErrorRecovery:
            franka_state_msg.robot_mode =
                franka_msgs::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
            break;
        }

        franka_state_msg.header.stamp = ros::Time::now();

        joint_states_pub_.publish(joint_states_msg);
        robot_states_pub_.publish(franka_state_msg);
        loop_rate.sleep();
    }
}

void franka_driver::halt(){
    state_pub_thread_->join();
    if(mode_==1)
        servoj_thread_->join();
}

void franka_driver::servojInterface(const franka_msgs::servoj::ConstPtr& msg){
    if(msg->keepalive==1 && msg->cmd_q.size()==7){
        for(unsigned int i=0;i<7;i++)
            servoj_cmd_[i] = msg->cmd_q[i];
        servoj_control_active_ = true;
    }
    else
        servoj_control_active_ = false;
}

bool franka_driver::errorRecovery(franka_msgs::error_recovery::Request &req,
                   franka_msgs::error_recovery::Response &res){
    p_robot_->automaticErrorRecovery();
    res.result = true;
    return true;
}

void franka_driver::servoj_thread_func(){
    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if(servoj_control_active_ && readOnce_shutdown){
            try{
                double time = 0.0;
                p_robot_->control([this,&time](const franka::RobotState& robot_state,
                                                franka::Duration period) -> franka::JointVelocities {
                    robot_state_ = robot_state;
                    time += period.toSec();

                    double vel_cmd[7];
                    double k_scale = 10;
                    for(unsigned int i=0;i<7;i++){
                        vel_cmd[i] = (servoj_cmd_[i]-robot_state_.q[i])*k_scale;
                    }

                    franka::JointVelocities velocities = {{vel_cmd[0],vel_cmd[1],vel_cmd[2],
                                                          vel_cmd[3],vel_cmd[4],vel_cmd[5],
                                                          vel_cmd[6]}};

                    if (!servoj_control_active_){
                        franka::JointVelocities velocities_z = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
                        std::cout << std::endl << "Finished motion" << std::endl;
                        return franka::MotionFinished(velocities_z);
                    }
                    return velocities;
                },franka::ControllerMode::kJointImpedance,true,10);
                std::cout << std::endl << "control shutdown" << std::endl;
                readOnce_shutdown = false;
            }
            catch (const franka::ControlException& e){
                std::cout << e.what() << std::endl;
                std::cout << "Running error recovery..." << std::endl;
                p_robot_->automaticErrorRecovery();
                readOnce_shutdown = false;
            }
        }
        else
            loop_rate.sleep();
    }
}

void franka_driver::executeCB(actionlib::ServerGoalHandle<
                              control_msgs::FollowJointTrajectoryAction> gh){
    if(has_goal_){
        result_.error_string = "Cannot accept a new trajectory when a trajectory is being executing!";
        ROS_ERROR("Cannot accept a new trajectory when a trajectory is being executing!");
        gh.setRejected(result_, result_.error_string);
        return;
    }

    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *gh.getGoal();
    goal_handle_ = gh;

    if (!start_positions_match(goal.trajectory, 0.01)) {
        result_.error_string = "Goal start doesn't match current pose";
        ROS_ERROR("Goal start doesn't match current pose");
        gh.setRejected(result_, result_.error_string);
        return;
    }

    std::vector<double> timestamps;
    std::vector<std::vector<double> > positions, velocities, acceleration;
    bool trajectory_modified = false;
    if (goal.trajectory.points[0].time_from_start.toSec() != 0.) {
        ROS_WARN(
                "Trajectory's first point should be the current position, with time_from_start set to 0.0 - Inserting point in malformed trajectory");
        timestamps.push_back(0.0);
        std::vector<double> current_position,current_velocity,current_acceleration;
        for(unsigned int i=0;i<7;i++){
            current_position.push_back(robot_state_.q_d[i]);
            current_velocity.push_back(0.0);
            current_acceleration.push_back(0.0);
        }
        positions.push_back(current_position);
        velocities.push_back(current_velocity);
        acceleration.push_back(current_acceleration);
        trajectory_modified = true;
    }
    for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
        timestamps.push_back(goal.trajectory.points[i].time_from_start.toSec());
        if(i==0 && trajectory_modified==false){
            std::vector<double> position_d;
            for(unsigned int k=0;k<7;k++)
                position_d.push_back(robot_state_.q_d[k]);
            positions.push_back(position_d);
        }
        else
            positions.push_back(goal.trajectory.points[i].positions);

        velocities.push_back(goal.trajectory.points[i].velocities);
        acceleration.push_back(goal.trajectory.points[i].accelerations);
    }

    has_goal_ = true;
    reach_goal_ = false;
    goal_handle_.setAccepted();
    std::thread(&franka_driver::trajThread, this, timestamps, positions,
            velocities,acceleration).detach();
}

bool franka_driver::start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps){
    for (unsigned int i = 0; i < traj.points[0].positions.size(); i++){
        if( fabs(traj.points[0].positions[i] - robot_state_.q[i]) > eps ){
            return false;
        }
    }
    return true;
}

void franka_driver::trajThread(std::vector<double> timestamps,
        std::vector<std::vector<double> > positions,
        std::vector<std::vector<double> > velocities,
        std::vector<std::vector<double> > acceleration) {
    doTraj(timestamps, positions, velocities, acceleration);
    if (reach_goal_) {
        result_.error_code = result_.SUCCESSFUL;
        goal_handle_.setSucceeded(result_);
        has_goal_ = false;
    }
}

void franka_driver::doTraj(std::vector<double> inp_timestamps,
                           std::vector<std::vector<double> > inp_positions,
                           std::vector<std::vector<double> > inp_velocities,
                           std::vector<std::vector<double> > inp_acceleration){
    trajectory_control_active_ = true;
    while(!readOnce_shutdown){
        ROS_INFO("Waiting readOnce shutDown...");
    }

    try{
        double time = 0.0;
        int j=0;
        p_robot_->control([this,&time,&j,&inp_timestamps,&inp_positions,&inp_velocities,&inp_acceleration](const franka::RobotState& robot_state,
                                                 franka::Duration period) -> franka::JointPositions {
            robot_state_ = robot_state;
            time += period.toSec();

            if (time>inp_timestamps[inp_timestamps.size() - 1]){
                unsigned int trajectory_size = inp_timestamps.size();
                franka::JointPositions finish_point = {{inp_positions[trajectory_size-1][0],
                                                        inp_positions[trajectory_size-1][1],
                                                        inp_positions[trajectory_size-1][2],
                                                        inp_positions[trajectory_size-1][3],
                                                        inp_positions[trajectory_size-1][4],
                                                        inp_positions[trajectory_size-1][5],
                                                        inp_positions[trajectory_size-1][6]}};

                if((time-inp_timestamps[inp_timestamps.size() - 1])<=0.1)
                    return finish_point;
                else{
                    std::cout << std::endl << "Finished motion" << std::endl;
                    return franka::MotionFinished(finish_point);
                }
            }

            while (inp_timestamps[j]
                    <= time && j < inp_timestamps.size() - 1) {
                j += 1;
            }

            std::vector<double> cmd_positions = interp_fifth(
                    time - inp_timestamps[j - 1],
                    inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
                    inp_positions[j], inp_velocities[j - 1], inp_velocities[j],
                    inp_acceleration[j-1],inp_acceleration[j]);

            franka::JointPositions output = {{cmd_positions[0], cmd_positions[1],
                                              cmd_positions[2], cmd_positions[3],
                                              cmd_positions[4], cmd_positions[5],
                                              cmd_positions[6]}};
            return output;
        },franka::ControllerMode::kJointImpedance,true,100);

        std::cout << std::endl << "trajectory control shutdown" << std::endl;
        readOnce_shutdown = false;
        trajectory_control_active_ = false;
        reach_goal_ = true;
    }
    catch (const franka::ControlException& e){
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        p_robot_->automaticErrorRecovery();
        readOnce_shutdown = false;
        trajectory_control_active_ = false;
        reach_goal_ = true;
    }
}


std::vector<double> franka_driver::interp_cubic(double t, double T,
        std::vector<double> p0_pos, std::vector<double> p1_pos,
        std::vector<double> p0_vel, std::vector<double> p1_vel) {
    std::vector<double> positions;
    for (unsigned int i = 0; i < p0_pos.size(); i++) {
        double a = p0_pos[i];
        double b = p0_vel[i];
        double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
                - T * p1_vel[i]) / pow(T, 2);
        double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
                + T * p1_vel[i]) / pow(T, 3);
        positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
    }
    return positions;
}

std::vector<double> franka_driver::interp_fifth(double t, double T,
        std::vector<double> p0_pos, std::vector<double> p1_pos,
        std::vector<double> p0_vel, std::vector<double> p1_vel,
        std::vector<double> p0_acc, std::vector<double> p1_acc) {
    std::vector<double> positions;
    for (unsigned int i = 0; i < p0_pos.size(); i++) {
        double a0 = p0_pos[i];
        double a1 = p0_vel[i];
        double a2 = p0_acc[i]/2;
        double a3 = (20*p1_pos[i]-20*p0_pos[i]-(8*p1_vel[i]+12*p0_vel[i])*T-(3*p0_acc[i]-p1_acc[i])*T*T)/(2*pow(T, 3));
        double a4 = (30*p0_pos[i]-30*p1_pos[i]+(14*p1_vel[i]+16*p0_vel[i])*T+(3*p0_acc[i]-2*p1_acc[i])*T*T)/(2*pow(T, 4));
        double a5 = (12*p1_pos[i]-12*p0_pos[i]-(6*p1_vel[i]+6*p0_vel[i])*T-(p0_acc[i]-p1_acc[i])*T*T)/(2*pow(T, 5));
        positions.push_back(a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t,4) + a5 * pow(t,5));
    }
    return positions;
}
