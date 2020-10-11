#include <servo_control_class/servo_control.h>

franka_teleoperation::franka_teleoperation(std::string robot_name_, std::string master_name_) : 
robot_name(robot_name_), 
master_name(master_name_)
{
    nh_.getParam("/urdf_path", urdf_file);
    nh_.getParam("/base_frame_"+robot_name, chain_start);
    nh_.getParam("/end_frame_"+robot_name, chain_end);

    nh_.getParam("/sim", sim);
    nh_.getParam("/low_pass_fliter",filter);   
    //nh_.getParam("/rcm_x_"+robot_name, rcm_point[0]);
    //nh_.getParam("/rcm_y_"+robot_name, rcm_point[1]);
    //nh_.getParam("/rcm_z_"+robot_name, rcm_point[2]);
    nh_.getParam("/rcmThreshold_"+robot_name, rcmErrorThreshold);
    nh_.getParam("/keyboard",keyBoard);
    nh_.getParam("/footswitch",footswitch);
    nh_.getParam("/camera_in_base1_x_"+robot_name, camera_in_base1[0]);
    nh_.getParam("/camera_in_base1_y_"+robot_name, camera_in_base1[1]);
    nh_.getParam("/camera_in_base1_z_"+robot_name, camera_in_base1[2]);
    nh_.getParam("/camera_in_base1_qx_"+robot_name, camera_in_base1[3]);
    nh_.getParam("/camera_in_base1_qy_"+robot_name, camera_in_base1[4]);
    nh_.getParam("/camera_in_base1_qz_"+robot_name, camera_in_base1[5]);
    nh_.getParam("/camera_in_base1_qw_"+robot_name, camera_in_base1[6]);
    nh_.getParam("/eye_hand_cooperation", eye_hand_cooperation);
    nh_.getParam("/rcm", rcm_enable);
    nh_.getParam("/master", master_device);
    nh_.getParam("/usefootsitch", useFootswitch);

    std::ifstream ifs("/home/master/franka_final_ws/src/rcm.txt");
    std::string str;
    if(robot_name == "franka1")
    {
		ifs>>str;
        rcm_point[0] = std::stod(str);
		ifs>>str;
        rcm_point[1] = std::stod(str);
		ifs>>str;
        rcm_point[2] = std::stod(str);
    }else if(robot_name == "franka2")
    {
		ifs>>str;
		ifs>>str;
		ifs>>str;
		ifs>>str;
        rcm_point[0] = std::stod(str);
		ifs>>str;
        rcm_point[1] = std::stod(str);
		ifs>>str;
        rcm_point[2] = std::stod(str);
	}
    ifs.close();

    //if use eyeHandCooperation then read cam pos from config
    if(eye_hand_cooperation){
        Eigen::Quaterniond cam_in_base1_q(camera_in_base1[6],camera_in_base1[3],camera_in_base1[4],camera_in_base1[5]);
        Eigen::Matrix<double,3,3> cam_in_base1_r = cam_in_base1_q.toRotationMatrix();
        for(unsigned int i=0;i<3;i++){
            for(unsigned int j=0;j<3;j++){
                camera_in_base1_matrix(i,j) = cam_in_base1_r(i,j);
            }
        }
        for(unsigned int i=0;i<3;i++){
            camera_in_base1_matrix(i,3) = camera_in_base1[i];
        }
        for(unsigned int i=0;i<3;i++){
            camera_in_base1_matrix(3,i) = 0;
        }
        camera_in_base1_matrix(3,3) = 1;
    }else{//cam pos == franka base pos
        for(unsigned int i=0;i<4;i++)
            for(unsigned int j=0;j<4;j++){
                camera_in_base1_matrix(i,j) = 0;
            }
        camera_in_base1_matrix(0,0) = 1;
        camera_in_base1_matrix(1,1) = 1;
        camera_in_base1_matrix(2,2) = 1;
        camera_in_base1_matrix(3,3) = 1;
    }
    //screen in cam
    //in screen forward is x,left is y,up is z. screen system is the same as master device
    //in cam forward is z,right is x,down is y
    cam_to_screen(0,0)=0;	cam_to_screen(0,1)=-1;	cam_to_screen(0,2)=0;	cam_to_screen(0,3)=0;
	cam_to_screen(1,0)=0;	cam_to_screen(1,1)=0;	cam_to_screen(1,2)=-1;	cam_to_screen(1,3)=0;
	cam_to_screen(2,0)=1;	cam_to_screen(2,1)=0;	cam_to_screen(2,2)=0;	cam_to_screen(2,3)=0;
	cam_to_screen(3,0)=0;	cam_to_screen(3,1)=0;	cam_to_screen(3,2)=0;	cam_to_screen(3,3)=1;

    std::cout<<"urdf_file: "<<urdf_file<<std::endl;
    std::cout<<"base_frame: "<<chain_start<<std::endl;
    std::cout<<"end_frame: "<<chain_end<<std::endl;

    std::cout<<"sim: "<<sim<<std::endl;
    std::cout<<"fliter: "<<filter<<std::endl;
    std::cout<<"eye_hand_cooperation: "<<eye_hand_cooperation<<std::endl;
    std::cout<<"rcm: "<<rcm_enable<<std::endl;
    std::cout<<"rcmThreshold: "<<rcmErrorThreshold<<std::endl;
    std::cout<<"keyBoard: "<<keyBoard<<std::endl;
    std::cout<<"footswitch: "<<footswitch<<std::endl;
    std::cout<<"rcm: "<<rcm_point[0]<<"   "<<rcm_point[1]<<"   "<<rcm_point[2]<<std::endl;
    std::cout<<"camera_in_base1_matrix:"<<std::endl<<camera_in_base1_matrix<<std::endl;

    if(!sim)
        pub_franka_script = nh_.advertise<franka_msgs::servoj>(robot_name+"/franka/servoj",10);
    else
        pub_sim_servoj = nh_.advertise<franka_msgs::servoj>(robot_name+"/sim_franka_servoj", 10);

    pub_tool = nh_.advertise<franka_msgs::servoj>(robot_name+"_tool_servoj",10);

    sub_joint_position = nh_.subscribe(robot_name+"/joint_states", 100, &franka_teleoperation::joint_position_callback, this);
    sub_tool_joints    = nh_.subscribe(robot_name+"_tool_states", 100, &franka_teleoperation::tool_position_callback, this);

    if(master_device == "touch"){
        if(!filter)
            sub_touch_map = nh_.subscribe("touch_map", 100, &franka_teleoperation::touch_map_callback, this);
        else
            sub_touch_map = nh_.subscribe("touch_map_filter", 100, &franka_teleoperation::touch_map_callback, this);
    }else{
		std::cout<<"omega"<<std::endl;
        if(!filter){
			std::cout<<"omega_map"<<std::endl;
            sub_omega_map = nh_.subscribe(master_name+"/omega_map", 100, &franka_teleoperation::omega_map_callback, this);
		}
        else{
			std::cout<<"omega_map_filter"<<std::endl;
            sub_omega_map = nh_.subscribe(master_name+"/omega_map_filter", 100, &franka_teleoperation::omega_map_callback, this);
		}
    }

    fk_thread_ = new std::thread(boost::bind(&franka_teleoperation::fk_func,this));// forward kinematic thread    
	keyboard_thread_ = new std::thread(boost::bind(&franka_teleoperation::keyboard_func,this));// read keyboard input thread
    footswitch_thread_ = new std::thread(boost::bind(&franka_teleoperation::footswitch_func,this));// read footswitch thread   
	control_thread_ = new std::thread(boost::bind(&franka_teleoperation::control_func,this));// control thread
}

franka_teleoperation::~franka_teleoperation(){}

void franka_teleoperation::halt(){
    fk_thread_->join();
    keyboard_thread_->join();
    footswitch_thread_->join();
    control_thread_->join();
}

//calculate davinci tool fk in it's frame
//DH matrix in franka_description/robot
void franka_teleoperation::tool_fk_in_toolFrame(){
    Eigen::Matrix<double,3,3> T0,T1,T2,T3,T4;
    T0<<0,-1,0,1,0,0,0,0,1;
    T1<<cos(tool_pos[0]),-1*sin(tool_pos[0]),0,sin(tool_pos[0]),cos(tool_pos[0]),0,0,0,1;
    T2<<-1*sin(tool_pos[1]),-1*cos(tool_pos[1]),0,0,0,1,-1*cos(tool_pos[1]),sin(tool_pos[1]),0;
    T3<<cos(tool_pos[2]),-1*sin(tool_pos[2]),0,0,0,1,-1*sin(tool_pos[2]),-1*cos(tool_pos[2]),0;
    T4<<0,0,-1,1,0,0,0,-1,0;
    
    curr_tool_orien_in_tool = T0*T1*T2*T3*T4;
}

void franka_teleoperation::fk_func(){
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile(urdf_file, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        exit(-1);
    }

    KDL::Chain my_chain;
    if(!my_tree.getChain(chain_start, chain_end, my_chain)){
        ROS_ERROR("Failed to convert to chain");
        exit(-1);
    }

    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(my_chain);
    nj = my_chain.getNrOfJoints();
    if(nj!=7){
        ROS_ERROR("nj error!");
        exit(-1);
    }
    KDL::JntArray jointpositions = KDL::JntArray(nj);

    ros::Rate loop_rate(100);
    while(ros::ok()){
        for(unsigned int i=0;i<nj;i++){
          jointpositions(i)=joint_positions[i];
        }

        KDL::Frame cartpos;

        int kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
        if(kinematics_status<0){
            ROS_ERROR("fk error!");
            exit(-1);
        }

        for(unsigned int i=0;i<3;i++)
            for(unsigned int j=0;j<3;j++){
                cur_ee(i,j) = cartpos.M(i,j);
            }
        cur_ee(0,3) = cartpos.p(0);cur_ee(1,3) = cartpos.p(1);cur_ee(2,3) = cartpos.p(2);
        cur_ee(3,0) = 0;cur_ee(3,1) = 0;cur_ee(3,2) = 0;cur_ee(3,3) = 1;

        slave1_current_pose = cur_ee;// get the fk of franka robot in franka system

        Eigen::Matrix<double,3,3> slave1_curr_orien;
        for(unsigned int i=0;i<3;i++){
            for(unsigned int j=0;j<3;j++){
                slave1_curr_orien(i,j) = slave1_current_pose(i,j);
            }
        }
        tool_fk_in_toolFrame();
        curr_tool_orien_in_base = slave1_curr_orien * curr_tool_orien_in_tool;//tool orientation in franka system

        loop_rate.sleep();
    }
    std::cout<<"exit fk thread"<<std::endl;
}

void franka_teleoperation::keyboard_func(){
    int keys_fd;
    struct input_event t;
    keys_fd = open (keyBoard.c_str(), O_RDONLY);
    if (keys_fd <= 0){
            ROS_ERROR("can't open keyboard device!");
            exit(-1);
    }

    while (ros::ok()) {
        if (read(keys_fd, &t, sizeof (t)) == sizeof (t)){
            if (t.code == KEY_E && useFootswitch==false){
                if(t.value==1){
                    master1_pos_zero[0] = master1_pos[0];
                    master1_pos_zero[1] = master1_pos[1];
                    master1_pos_zero[2] = master1_pos[2];
                    master1_rpy_zero[0] = master1_rpy[0];
                    master1_rpy_zero[1] = master1_rpy[1];
                    master1_rpy_zero[2] = master1_rpy[2];

                    slave1_zero = slave1_current_pose;
					slave1_zero_in_cam = camera_in_base1_matrix.inverse() * slave1_current_pose;

                    if(eye_hand_cooperation){
					    slave1_zero_in_cam = cam_to_screen.inverse()*slave1_zero_in_cam;
                        Eigen::Matrix<double,3,3> cam_in_base1_orien,screen_in_cam_orien;
                        for(unsigned int i=0;i<3;i++){
                            for(unsigned int j=0;j<3;j++){
                                cam_in_base1_orien(i,j) = camera_in_base1_matrix(i,j);
                                screen_in_cam_orien(i,j) = cam_to_screen(i,j);
                            }
                        }
                        curr_tool_orien_in_cam = screen_in_cam_orien.inverse() * cam_in_base1_orien.inverse() * curr_tool_orien_in_base;
                    }

                    tool_move = 0;
					rcm_ready = false;

                    std::cout<<"keyboard master and slave init success"<<std::endl;
                }else if(t.value==2){
                    control_activate_ = true;
                    //std::cout<<"activate controller"<<std::endl;
                }else if(t.value==0){
                    std::cout<<"keyboard deactivate controller"<<std::endl;
                    control_activate_ = false;
                }
            }else if(t.code == KEY_0 && t.value==1){ //mode 0
                control_activate_ = false;
                mode = 0;
				if(rcm_enable){
					scale_p_x = 0.2;
					scale_p_y = 0.2;
					scale_p_z = 0.2;
				}
                std::cout<<"***********************"<<std::endl;
                std::cout<<"position mode!"<<std::endl;
            }else if(t.code == KEY_1 && t.value==1){//mode 1
                control_activate_ = false;
                mode = 1;
                std::cout<<"***********************"<<std::endl;
                std::cout<<"posture mode in base frame!"<<std::endl;
            }else if(t.code == KEY_2 && t.value==1){//mode 2
                control_activate_ = false;
                mode = 2;
                std::cout<<"***********************"<<std::endl;
                std::cout<<"pose mode in base frame!"<<std::endl;
            }else if(t.code == KEY_3 && t.value==1){//mode 3
                control_activate_ = false;
                mode = 3;
                std::cout<<"***********************"<<std::endl;
                std::cout<<"tool +"<<std::endl;
            }else if(t.code == KEY_4 && t.value==1){//mode 4
                control_activate_ = false;
                mode = 4;
                std::cout<<"***********************"<<std::endl;
                std::cout<<"tool -"<<std::endl;
            }else if(t.code == KEY_W && t.value==1){//increase position speed
                control_activate_ = false;
                if(scale_p_x<(scale_p_max-scale_p_step)){
                    scale_p_x += scale_p_step;
                    scale_p_y += scale_p_step;
                    scale_p_z += scale_p_step;
                }else{
                    scale_p_x = scale_p_max;
                    scale_p_y = scale_p_max;
                    scale_p_z = scale_p_max;
                }
                std::cout<<"***********************"<<std::endl;
                std::cout<<"position_x_scale:  "<<scale_p_x<<"  position_y_scale: "<<scale_p_y
                         <<"  position_z_scale: "<<scale_p_z<<std::endl;
            }else if(t.code == KEY_S && t.value==1){//decrease position speed
                control_activate_ = false;
                if(scale_p_x>(scale_p_min+scale_p_step)){
                    scale_p_x -= scale_p_step;
                    scale_p_y -= scale_p_step;
                    scale_p_z -= scale_p_step;
                }else{
                    scale_p_x = scale_p_min;
                    scale_p_y = scale_p_min;
                    scale_p_z = scale_p_min;
                }
                std::cout<<"***********************"<<std::endl;
                std::cout<<"position_x_scale:  "<<scale_p_x<<"  position_y_scale: "<<scale_p_y
                         <<"  position_z_scale: "<<scale_p_z<<std::endl;
            }else if(t.code == KEY_D && t.value==1){//increase orientation speed
                control_activate_ = false;
                if(scale_r_x<(scale_r_max-scale_r_step)){
                    scale_r_x += scale_r_step;
                    scale_r_y += scale_r_step;
                    scale_r_z += scale_r_step;
                }else{
                    scale_r_x = scale_r_max;
                    scale_r_y = scale_r_max;
                    scale_r_z = scale_r_max;
                }
                std::cout<<"***********************"<<std::endl;
                std::cout<<"posture_x_scale:  "<<scale_r_x<<"  posture_y_scale: "<<scale_r_y
                         <<"  posture_z_scale: "<<scale_r_z<<std::endl;
            }else if(t.code == KEY_A && t.value==1){//decrease orientation speed
                control_activate_ = false;
                if(scale_r_x>(scale_r_min+scale_r_step)){
                    scale_r_x -= scale_r_step;
                    scale_r_y -= scale_r_step;
                    scale_r_z -= scale_r_step;
                }else{
                    scale_r_x = scale_r_min;
                    scale_r_y = scale_r_min;
                    scale_r_z = scale_r_min;
                }
                std::cout<<"***********************"<<std::endl;
                std::cout<<"posture_x_scale:  "<<scale_r_x<<"  posture_y_scale: "<<scale_r_y
                         <<"  posture_z_scale: "<<scale_r_z<<std::endl;
            }
        }
    }
    close (keys_fd);

    std::cout<<"exit keyboard thread"<<std::endl;
}

void franka_teleoperation::footswitch_func(){
    int keys_fd;
    struct input_event t;
    keys_fd = open (footswitch.c_str(), O_RDONLY);
    if (keys_fd <= 0){
            ROS_ERROR("can't open footswitch device!");
            exit(-1);
    }

    while (ros::ok()) {
        if (read(keys_fd, &t, sizeof (t)) == sizeof (t)){
            if (t.code == KEY_E && useFootswitch){//enable button event
                if(t.value==1){// first time press enable button, do init work
                    master1_pos_zero[0] = master1_pos[0];
                    master1_pos_zero[1] = master1_pos[1];
                    master1_pos_zero[2] = master1_pos[2];
                    master1_rpy_zero[0] = master1_rpy[0];
                    master1_rpy_zero[1] = master1_rpy[1];
                    master1_rpy_zero[2] = master1_rpy[2];

                    slave1_zero = slave1_current_pose;
                    slave1_zero_in_cam = camera_in_base1_matrix.inverse() * slave1_current_pose;

                    if(eye_hand_cooperation){
					    slave1_zero_in_cam = cam_to_screen.inverse()*slave1_zero_in_cam;
                        Eigen::Matrix<double,3,3> cam_in_base1_orien,screen_in_cam_orien;
                        for(unsigned int i=0;i<3;i++){
                            for(unsigned int j=0;j<3;j++){
                                cam_in_base1_orien(i,j) = camera_in_base1_matrix(i,j);
                                screen_in_cam_orien(i,j) = cam_to_screen(i,j);
                            }
                        }
                        curr_tool_orien_in_cam = screen_in_cam_orien.inverse() * cam_in_base1_orien.inverse() * curr_tool_orien_in_base;
                    }
                    tool_move = 0;
					rcm_ready = false;
                    std::cout<<"footswitch master and slave init success"<<std::endl;
                }else if(t.value==2){// continue press enable button, robot start move
                    control_activate_ = true;
                    //std::cout<<"activate controller"<<std::endl;
                }else if(t.value==0){// release enable button
                    std::cout<<"footswitch deactivate controller"<<std::endl;
                    control_activate_ = false;
                }
            }
        }
    }
    close (keys_fd);

    std::cout<<"exit footswitch thread"<<std::endl;
}

void franka_teleoperation::control_func(){
/*
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    int count = 0;
    tf::StampedTransform transform;
    while (count<50){
    
    try{
      listener.lookupTransform(chain_start, chain_end,  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    count++;
	}
    std::cout<<transform.getOrigin().x()<<"  "<<transform.getOrigin().y()<<"   "<<transform.getOrigin().z()<<std::endl;
    std::cout<<transform.getOrigin().x()<<"  "<<transform.getOrigin().y()<<"   "<<transform.getOrigin().z()<<std::endl;
    std::cout<<transform.getOrigin().x()<<"  "<<transform.getOrigin().y()<<"   "<<transform.getOrigin().z()<<std::endl;
    std::cout<<transform.getOrigin().x()<<"  "<<transform.getOrigin().y()<<"   "<<transform.getOrigin().z()<<std::endl;
    rcm_point[0] = transform.getOrigin().x();
	rcm_point[1] = transform.getOrigin().y();
	rcm_point[2] = transform.getOrigin().z();
*/
    double timeout = 0.005;
    urdf_param = "/robot_description";
    double eps = 1e-5;
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
    TRAC_IK::TRAC_IK rcmSolver(chain_start, chain_end, rcm_point, rcmErrorThreshold, urdf_param, timeout, 1e-6);
    KDL::Chain my_chain,rcm_chain;
    KDL::JntArray ll, ul, rcm_ll, rcm_ul;

    bool valid = tracik_solver.getKDLChain(my_chain);
    if (!valid) {
      ROS_ERROR("There was no valid KDL chain found in tracik_solver");
      exit(-1);
    }

    valid = tracik_solver.getKDLLimits(ll,ul);
    if (!valid) {
      ROS_ERROR("There were no valid KDL joint limits found in tracik_solver");
      exit(-1);
    }

    valid = rcmSolver.getKDLChain(rcm_chain);
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
    double tool_pos_to_slove[3];
    tool_pos_to_slove[0] = 0;   tool_pos_to_slove[1] = 0;   tool_pos_to_slove[2] = 0;

    for(int i=0;i<7;i++){
		jointpositions_to_solve(i) = joint_positions[i];
	}

    ros::Rate loop_rate(100);
    while(ros::ok()){
        if(control_activate_){
    		for(int i=0;i<7;i++){
				jointpositions_to_solve(i) = joint_positions[i];
			}
            for(int i=0;i<3;i++){
                tool_pos_to_slove[i] = tool_pos[i];
            }

            Eigen::Matrix<double,4,4> slave1_desire_pose_in_cam = slave1_zero_in_cam;
            slave1_desire_pose_in_cam(0,3) = direction1_x * (master1_pos[0]-master1_pos_zero[0]) * scale_p_x + slave1_zero_in_cam(0,3);
            slave1_desire_pose_in_cam(1,3) = direction1_y * (master1_pos[1]-master1_pos_zero[1]) * scale_p_y + slave1_zero_in_cam(1,3);
            slave1_desire_pose_in_cam(2,3) = direction1_z * (master1_pos[2]-master1_pos_zero[2]) * scale_p_z + slave1_zero_in_cam(2,3);

            double master1_rpy_x_increase = direction1_rpy_r * scale_r_x*(master1_rpy[0]-master1_rpy_zero[0]);
            double master1_rpy_y_increase = direction1_rpy_p * scale_r_y*(master1_rpy[1]-master1_rpy_zero[1]);
            double master1_rpy_z_increase = direction1_rpy_y * scale_r_z*(master1_rpy[2]-master1_rpy_zero[2]);
            //std::cout<<master1_rpy_x_increase<<"  "<<master1_rpy_y_increase<<"  "<<master1_rpy_z_increase<<std::endl;

            Eigen::Matrix<double,4,4> rotx,roty,rotz;
            rotx(0,0) = 1;rotx(0,1) = 0;                             rotx(0,2) = 0;                               rotx(0,3) = 0;
            rotx(1,0) = 0;rotx(1,1) = cos(master1_rpy_x_increase);   rotx(1,2) = -1*sin(master1_rpy_x_increase);  rotx(1,3) = 0;
            rotx(2,0) = 0;rotx(2,1) = sin(master1_rpy_x_increase);   rotx(2,2) = cos(master1_rpy_x_increase);     rotx(2,3) = 0;
            rotx(3,0) = 0;rotx(3,1) = 0;                             rotx(3,2) = 0;                               rotx(3,3) = 1;

            roty(0,0) = cos(master1_rpy_y_increase);     roty(0,1) = 0;  roty(0,2) = sin(master1_rpy_y_increase); roty(0,3) = 0;
            roty(1,0) = 0;                               roty(1,1) = 1;  roty(1,2) = 0;                           roty(1,3) = 0;
            roty(2,0) = -1*sin(master1_rpy_y_increase);  roty(2,1) = 0;  roty(2,2) = cos(master1_rpy_y_increase); roty(2,3) = 0;
            roty(3,0) = 0;                               roty(3,1) = 0;  roty(3,2) = 0;                           roty(3,3) = 1;

            rotz(0,0) = cos(master1_rpy_z_increase); rotz(0,1) = -1*sin(master1_rpy_z_increase);  rotz(0,2) = 0;  rotz(0,3) = 0;
            rotz(1,0) = sin(master1_rpy_z_increase); rotz(1,1) = cos(master1_rpy_z_increase);     rotz(1,2) = 0;  rotz(1,3) = 0;
            rotz(2,0) = 0;                           rotz(2,1) = 0;                               rotz(2,2) = 1;  rotz(2,3) = 0;
            rotz(3,0) = 0;                           rotz(3,1) = 0;                               rotz(3,2) = 0;  rotz(3,3) = 1;

            //rot in cam/screen frame
            Eigen::Matrix<double,4,4> slave1_desire_pose_fix_pos_in_cam;
            slave1_desire_pose_fix_pos_in_cam = rotz * roty * rotx * slave1_zero_in_cam;

            //tool_roate in cam/screen frame
            Eigen::Matrix<double,3,3> rotx3,roty3,rotz3;
            for(unsigned int i=0;i<3;i++){
                for(unsigned int j=0;j<3;j++){
                    rotx3(i,j) = rotx(i,j);
                    roty3(i,j) = roty(i,j);
                    rotz3(i,j) = rotz(i,j);
                }
            }
            Eigen::Matrix<double,3,3> tool1_desire_orien_in_cam;
            tool1_desire_orien_in_cam = rotz3 * roty3 * rotx3 * curr_tool_orien_in_cam;
            Eigen::Matrix<double,3,3> tool1_desire_orien_in_tool,screen_in_cam_orien,cam_in_base1_orien,slave1_curr_orien;
            for(unsigned int i=0;i<3;i++){
                for(unsigned int j=0;j<3;j++){
                    screen_in_cam_orien(i,j) = cam_to_screen(i,j);
                    cam_in_base1_orien(i,j) = camera_in_base1_matrix(i,j);
                    slave1_curr_orien(i,j) = slave1_current_pose(i,j);
                }
            }
            tool1_desire_orien_in_tool = slave1_curr_orien.inverse() * cam_in_base1_orien * screen_in_cam_orien * tool1_desire_orien_in_cam;
			//std::cout<<tool1_desire_orien_in_tool<<std::endl;
            //tool_roate in cam/screen frame

            Eigen::Matrix<double,4,4> slave1_desire_pose,slave1_desire_pose_fix_pos;
            if(eye_hand_cooperation){
                slave1_desire_pose = camera_in_base1_matrix * cam_to_screen * slave1_desire_pose_in_cam;
                slave1_desire_pose_fix_pos = camera_in_base1_matrix * cam_to_screen * slave1_desire_pose_fix_pos_in_cam;
            }
            else{
                slave1_desire_pose = camera_in_base1_matrix * slave1_desire_pose_in_cam;
                slave1_desire_pose_fix_pos = camera_in_base1_matrix * slave1_desire_pose_fix_pos_in_cam;
            }

            int result1 = -1;
            if(mode == 0 && !rcm_enable){//position mode
                KDL::JntArray q_init = KDL::JntArray(nj);
                for(int i=0; i<nj; i++){
                  q_init(i)=joint_positions[i];
                }

                KDL::Frame F_dest1;
                F_dest1.M(0,0) = slave1_desire_pose(0,0);
                F_dest1.M(0,1) = slave1_desire_pose(0,1);
                F_dest1.M(0,2) = slave1_desire_pose(0,2);

                F_dest1.M(1,0) = slave1_desire_pose(1,0);
                F_dest1.M(1,1) = slave1_desire_pose(1,1);
                F_dest1.M(1,2) = slave1_desire_pose(1,2);

                F_dest1.M(2,0) = slave1_desire_pose(2,0);
                F_dest1.M(2,1) = slave1_desire_pose(2,1);
                F_dest1.M(2,2) = slave1_desire_pose(2,2);

                F_dest1.p(0) = slave1_desire_pose(0,3);
                F_dest1.p(1) = slave1_desire_pose(1,3);
                F_dest1.p(2) = slave1_desire_pose(2,3);

                result1 = tracik_solver.CartToJnt(q_init,F_dest1,jointpositions_to_solve);
            }else if(mode == 1 && !rcm_enable){//rot in base frame
                KDL::JntArray q_init = KDL::JntArray(nj);
                for(int i=0; i<nj; i++){
                  q_init(i)=joint_positions[i];
                }

                KDL::Frame F_dest1;
                F_dest1.M(0,0) = slave1_desire_pose_fix_pos(0,0);
                F_dest1.M(0,1) = slave1_desire_pose_fix_pos(0,1);
                F_dest1.M(0,2) = slave1_desire_pose_fix_pos(0,2);

                F_dest1.M(1,0) = slave1_desire_pose_fix_pos(1,0);
                F_dest1.M(1,1) = slave1_desire_pose_fix_pos(1,1);
                F_dest1.M(1,2) = slave1_desire_pose_fix_pos(1,2);

                F_dest1.M(2,0) = slave1_desire_pose_fix_pos(2,0);
                F_dest1.M(2,1) = slave1_desire_pose_fix_pos(2,1);
                F_dest1.M(2,2) = slave1_desire_pose_fix_pos(2,2);

                F_dest1.p(0) = slave1_zero(0,3);
                F_dest1.p(1) = slave1_zero(1,3);
                F_dest1.p(2) = slave1_zero(2,3);
                
                result1 = tracik_solver.CartToJnt(q_init,F_dest1,jointpositions_to_solve);
            }else if(mode == 2 && !rcm_enable){//position mode + rot in base frame
                KDL::JntArray q_init = KDL::JntArray(nj);
                for(int i=0; i<nj; i++){
                  q_init(i)=joint_positions[i];
                }

                KDL::Frame F_dest1;
                F_dest1.M(0,0) = slave1_desire_pose_fix_pos(0,0);
                F_dest1.M(0,1) = slave1_desire_pose_fix_pos(0,1);
                F_dest1.M(0,2) = slave1_desire_pose_fix_pos(0,2);

                F_dest1.M(1,0) = slave1_desire_pose_fix_pos(1,0);
                F_dest1.M(1,1) = slave1_desire_pose_fix_pos(1,1);
                F_dest1.M(1,2) = slave1_desire_pose_fix_pos(1,2);

                F_dest1.M(2,0) = slave1_desire_pose_fix_pos(2,0);
                F_dest1.M(2,1) = slave1_desire_pose_fix_pos(2,1);
                F_dest1.M(2,2) = slave1_desire_pose_fix_pos(2,2);

                F_dest1.p(0) = slave1_desire_pose(0,3);
                F_dest1.p(1) = slave1_desire_pose(1,3);
                F_dest1.p(2) = slave1_desire_pose(2,3);

                result1 = tracik_solver.CartToJnt(q_init,F_dest1,jointpositions_to_solve);
            }else if(mode == 3){//tool feed
                Eigen::Matrix<double,4,4> zero_to_desire;
                tool_move += tool_step;
                zero_to_desire(0,0) = 1;zero_to_desire(0,1) = 0;zero_to_desire(0,2) = 0;zero_to_desire(0,3) = 0;
                zero_to_desire(1,0) = 0;zero_to_desire(1,1) = 1;zero_to_desire(1,2) = 0;zero_to_desire(1,3) = 0;
                zero_to_desire(2,0) = 0;zero_to_desire(2,1) = 0;zero_to_desire(2,2) = 1;zero_to_desire(2,3) = tool_move;
                zero_to_desire(3,0) = 0;zero_to_desire(3,1) = 0;zero_to_desire(3,2) = 0;zero_to_desire(3,3) = 1;

                Eigen::Matrix<double,4,4> tool_desired = slave1_zero*zero_to_desire;
                KDL::JntArray q_init = KDL::JntArray(nj);
                for(int i=0; i<nj; i++){
                  q_init(i)=joint_positions[i];
                }

                KDL::Frame F_dest;
                F_dest.M(0,0) = tool_desired(0,0);
                F_dest.M(0,1) = tool_desired(0,1);
                F_dest.M(0,2) = tool_desired(0,2);

                F_dest.M(1,0) = tool_desired(1,0);
                F_dest.M(1,1) = tool_desired(1,1);
                F_dest.M(1,2) = tool_desired(1,2);

                F_dest.M(2,0) = tool_desired(2,0);
                F_dest.M(2,1) = tool_desired(2,1);
                F_dest.M(2,2) = tool_desired(2,2);

                F_dest.p(0) = tool_desired(0,3);
                F_dest.p(1) = tool_desired(1,3);
                F_dest.p(2) = tool_desired(2,3);

                result1 = tracik_solver.CartToJnt(q_init,F_dest,jointpositions_to_solve);
            }else if(mode == 4){//tool back
                Eigen::Matrix<double,4,4> zero_to_desire;
                tool_move -= tool_step;
                zero_to_desire(0,0) = 1;zero_to_desire(0,1) = 0;zero_to_desire(0,2) = 0;zero_to_desire(0,3) = 0;
                zero_to_desire(1,0) = 0;zero_to_desire(1,1) = 1;zero_to_desire(1,2) = 0;zero_to_desire(1,3) = 0;
                zero_to_desire(2,0) = 0;zero_to_desire(2,1) = 0;zero_to_desire(2,2) = 1;zero_to_desire(2,3) = tool_move;
                zero_to_desire(3,0) = 0;zero_to_desire(3,1) = 0;zero_to_desire(3,2) = 0;zero_to_desire(3,3) = 1;

                Eigen::Matrix<double,4,4> tool_desired = slave1_zero*zero_to_desire;
                KDL::JntArray q_init = KDL::JntArray(nj);
                for(int i=0; i<nj; i++){
                  q_init(i)=joint_positions[i];
                }

                KDL::Frame F_dest;
                F_dest.M(0,0) = tool_desired(0,0);
                F_dest.M(0,1) = tool_desired(0,1);
                F_dest.M(0,2) = tool_desired(0,2);

                F_dest.M(1,0) = tool_desired(1,0);
                F_dest.M(1,1) = tool_desired(1,1);
                F_dest.M(1,2) = tool_desired(1,2);

                F_dest.M(2,0) = tool_desired(2,0);
                F_dest.M(2,1) = tool_desired(2,1);
                F_dest.M(2,2) = tool_desired(2,2);

                F_dest.p(0) = tool_desired(0,3);
                F_dest.p(1) = tool_desired(1,3);
                F_dest.p(2) = tool_desired(2,3);

                result1 = tracik_solver.CartToJnt(q_init,F_dest,jointpositions_to_solve);
            }else if(mode == 0 && rcm_enable){//RCM
                if(!rcm_ik){
                    double end_to_rcm = sqrt(pow(rcm_point[0]-slave1_current_pose(0,3),2)
                                            +pow(rcm_point[1]-slave1_current_pose(1,3),2)
                                            +pow(rcm_point[2]-slave1_current_pose(2,3),2));
                    if(rcm_point[2]>slave1_current_pose(2,3) && end_to_rcm>= 0.08){
                        KDL::JntArray q_init = KDL::JntArray(nj);
                        for(int i=0; i<nj; i++){
                            q_init(i)=joint_positions[i];
                        }

                        KDL::Frame F_dest;
                        Eigen::Matrix<double,3,1> desired_x_orientation;
                        Eigen::Matrix<double,3,1> desired_y_orientation;
                        Eigen::Matrix<double,3,1> desired_z_orientation;
                        desired_z_orientation(0,0) = slave1_desire_pose(0,3) - rcm_point[0];
                        desired_z_orientation(1,0) = slave1_desire_pose(1,3) - rcm_point[1];
                        desired_z_orientation(2,0) = slave1_desire_pose(2,3) - rcm_point[2];

                        double temp = sqrt(desired_z_orientation(0,0)*desired_z_orientation(0,0)
                                        +desired_z_orientation(1,0)*desired_z_orientation(1,0)
                                        +desired_z_orientation(2,0)*desired_z_orientation(2,0));
                        desired_z_orientation(0,0) = desired_z_orientation(0,0) / temp;
                        desired_z_orientation(1,0) = desired_z_orientation(1,0) / temp;
                        desired_z_orientation(2,0) = desired_z_orientation(2,0) / temp;

                        Eigen::Matrix<double,3,1> slave_zero_x_orientation,slave_zero_y_orientation,slave_zero_z_orientation;
                        slave_zero_x_orientation(0,0) = slave1_zero(0,0);
                        slave_zero_x_orientation(1,0) = slave1_zero(1,0);
                        slave_zero_x_orientation(2,0) = slave1_zero(2,0);

                        slave_zero_y_orientation(0,0) = slave1_zero(0,1);
                        slave_zero_y_orientation(1,0) = slave1_zero(1,1);
                        slave_zero_y_orientation(2,0) = slave1_zero(2,1);

                        slave_zero_z_orientation(0,0) = slave1_zero(0,2);
                        slave_zero_z_orientation(1,0) = slave1_zero(1,2);
                        slave_zero_z_orientation(2,0) = slave1_zero(2,2);

                        Eigen::Matrix<double,3,1> rotation_axis = slave_zero_z_orientation.cross(desired_z_orientation);
                        temp = sqrt(rotation_axis(0,0)*rotation_axis(0,0)
                                +rotation_axis(1,0)*rotation_axis(1,0)
                                +rotation_axis(2,0)*rotation_axis(2,0));
                        rotation_axis(0,0) = rotation_axis(0,0) / temp;
                        rotation_axis(1,0) = rotation_axis(1,0) / temp;
                        rotation_axis(2,0) = rotation_axis(2,0) / temp;
                        double rotation_angle = acos(slave_zero_z_orientation.dot(desired_z_orientation));

                        Eigen::AngleAxisd V(rotation_angle, Eigen::Vector3d(rotation_axis(0,0),rotation_axis(1,0),rotation_axis(2,0)));
                        Eigen::Matrix<double,3,3> rotation_matrix = V.toRotationMatrix();

                        desired_y_orientation = rotation_matrix * slave_zero_y_orientation;
                        desired_x_orientation = rotation_matrix * slave_zero_x_orientation;

                        F_dest.M(0,0) = desired_x_orientation(0,0);
                        F_dest.M(0,1) = desired_y_orientation(0,0);
                        F_dest.M(0,2) = desired_z_orientation(0,0);

                        F_dest.M(1,0) = desired_x_orientation(1,0);
                        F_dest.M(1,1) = desired_y_orientation(1,0);
                        F_dest.M(1,2) = desired_z_orientation(1,0);

                        F_dest.M(2,0) = desired_x_orientation(2,0);
                        F_dest.M(2,1) = desired_y_orientation(2,0);
                        F_dest.M(2,2) = desired_z_orientation(2,0);

                        F_dest.p(0) = slave1_desire_pose(0,3);
                        F_dest.p(1) = slave1_desire_pose(1,3);
                        F_dest.p(2) = slave1_desire_pose(2,3);
                        result1 = tracik_solver.CartToJnt(q_init,F_dest,jointpositions_to_solve);

                        double rcm_distance = 0;
                        for(int i=0;i<7;i++)
                            rcm_distance += pow((jointpositions_to_solve(i)-joint_positions[i]),2);
                        rcm_distance = sqrt(rcm_distance);

                        //std::cout<<"result: "<<result1<<std::endl;
                        //std::cout<<"rcm_distance: "<<rcm_distance<<std::endl;
                        if(rcm_distance>0.005 && rcm_ready==false && result1>0){
                            while(rcm_distance > 0.005 && rcm_distance<=0.3){
                                for(int i=0;i<7;i++){
                                    jointpositions_to_solve(i) = joint_positions[i] + 2*(jointpositions_to_solve(i)-joint_positions[i])/3;
                                }
                                rcm_distance = 0;
                                for(int i=0;i<7;i++)
                                    rcm_distance += pow((jointpositions_to_solve(i)-joint_positions[i]),2);
                                rcm_distance = sqrt(rcm_distance);
                            }                   
                        }else if(result1>0){
                            rcm_ready = true;
                            std::cout<<"ready"<<std::endl;
                            while(rcm_distance > 0.2 && rcm_distance<=0.3){
                                for(int i=0;i<7;i++){
                                    jointpositions_to_solve(i) = joint_positions[i] + 2*(jointpositions_to_solve(i)-joint_positions[i])/3;
                                }
                                rcm_distance = 0;
                                for(int i=0;i<7;i++)
                                    rcm_distance += pow((jointpositions_to_solve(i)-joint_positions[i]),2);
                                rcm_distance = sqrt(rcm_distance);
                            }
                        }   

                        if(result1<=0){
                            std::cout<<"no ik, need move robot to a better position"<<std::endl;
                            for(int i=0;i<7;i++){
                                jointpositions_to_solve(i) = joint_positions[i];
                            }
                        }
                    }else{
                        std::cout<<"relative position between rcm and end is error"<<std::endl;
                        for(int i=0;i<7;i++){
                            jointpositions_to_solve(i) = joint_positions[i];
                        }
                    }
                }else{
                    KDL::JntArray q_init = KDL::JntArray(nj);
                    for(int i=0; i<nj; i++){
                      q_init(i)=joint_positions[i];
                    }

                    KDL::Frame F_dest1;
                    F_dest1.p(0) = slave1_desire_pose(0,3);
                    F_dest1.p(1) = slave1_desire_pose(1,3);
                    F_dest1.p(2) = slave1_desire_pose(2,3);

                    result1 = rcmSolver.CartToJntConstraint(q_init,F_dest1,jointpositions_to_solve);                    
                }				
/*
                double k2 = -1*asin(tool1_desire_orien_in_tool(2,1));
                double k3 = asin(tool1_desire_orien_in_tool(2,0)/cos(k2));
                double k1 = atan2(tool1_desire_orien_in_tool(0,1)/(-1*cos(k2)),tool1_desire_orien_in_tool(1,1)/cos(k2));
				//std::cout<<std::endl;
				//std::cout<<std::endl;
                tool_pos_to_slove[0] = k1;
                tool_pos_to_slove[1] = k2;
                tool_pos_to_slove[2] = k3;

                if(tool_pos_to_slove[0]>tool1_max){
                    ROS_WARN("tool_joint_1_result_exceed_max, force modify");
                    tool_pos_to_slove[0] = tool1_max;
                }else if(tool_pos_to_slove[0]<tool1_min){
                    ROS_WARN("tool_joint_1_result_exceed_min, force modify");
                    tool_pos_to_slove[0] = tool1_min;
                }

                if(tool_pos_to_slove[1]>tool2_max){
                    ROS_WARN("tool_joint_2_result_exceed_max, force modify");
                    tool_pos_to_slove[1] = tool2_max;
                }else if(tool_pos_to_slove[1]<tool2_min){
                    ROS_WARN("tool_joint_2_result_exceed_min, force modify");
                    tool_pos_to_slove[1] = tool2_min;
                }

                if(tool_pos_to_slove[2]>tool3_max){
                    ROS_WARN("tool_joint_3_result_exceed_max, force modify");
                    tool_pos_to_slove[2] = tool3_max;
                }else if(tool_pos_to_slove[2]<tool3_min){
                    ROS_WARN("tool_joint_3_result_exceed_min, force modify");
                    tool_pos_to_slove[2] = tool3_min;
                }
*/
                //std::cout<<tool_pos_to_slove[0]<<"   "<<tool_pos_to_slove[1]<<"   "<<tool_pos_to_slove[2]<<std::endl;
            }else{
                for(int i=0;i<7;i++){
                    jointpositions_to_solve(i) = joint_positions[i];
                }
                for(int i=0;i<3;i++){
                    tool_pos_to_slove[i] = tool_pos[i];
                }
            }

            double k2 = -1*asin(tool1_desire_orien_in_tool(2,1));
            double k3 = asin(tool1_desire_orien_in_tool(2,0)/cos(k2));
            double k1 = atan2(tool1_desire_orien_in_tool(0,1)/(-1*cos(k2)),tool1_desire_orien_in_tool(1,1)/cos(k2));
			//std::cout<<std::endl;
			//std::cout<<std::endl;
            tool_pos_to_slove[0] = k1;
            tool_pos_to_slove[1] = k2;
            tool_pos_to_slove[2] = k3;

            if(tool_pos_to_slove[0]>tool1_max){
                ROS_WARN("tool_joint_1_result_exceed_max, force modify");
                tool_pos_to_slove[0] = tool1_max;
            }else if(tool_pos_to_slove[0]<tool1_min){
                ROS_WARN("tool_joint_1_result_exceed_min, force modify");
                tool_pos_to_slove[0] = tool1_min;
            }

            if(tool_pos_to_slove[1]>tool2_max){
                ROS_WARN("tool_joint_2_result_exceed_max, force modify");
                tool_pos_to_slove[1] = tool2_max;
            }else if(tool_pos_to_slove[1]<tool2_min){
                ROS_WARN("tool_joint_2_result_exceed_min, force modify");
                tool_pos_to_slove[1] = tool2_min;
            }

            if(tool_pos_to_slove[2]>tool3_max){
                ROS_WARN("tool_joint_3_result_exceed_max, force modify");
                tool_pos_to_slove[2] = tool3_max;
            }else if(tool_pos_to_slove[2]<tool3_min){
                ROS_WARN("tool_joint_3_result_exceed_min, force modify");
                tool_pos_to_slove[2] = tool3_min;
            }



            double joint_distance = 0;
            double tool_distance = 0;
            for(int i=0;i<7;i++)
                joint_distance += pow((jointpositions_to_solve(i)-joint_positions[i]),2);
            joint_distance = sqrt(joint_distance);

            for(int i=0;i<3;i++)
                tool_distance += pow(tool_pos_to_slove[i]-tool_pos[i],2);
            tool_distance = sqrt(tool_distance);

//             std::cout<<"joint_distance: "<<joint_distance<<std::endl;
//             std::cout<<"tool_distance: "<<tool_distance<<std::endl;
             if(joint_distance<0.1 && tool_distance<0.16/*|| (joint_distance<0.2 && rcm_enable)*/){
                 if(!sim){
                    franka_msgs::servoj msg;
                    msg.keepalive = 1;
                    msg.cmd_q.resize(7);
                    for(unsigned int i=0;i<7;i++)
                        msg.cmd_q[i] = jointpositions_to_solve(i);

                    franka_msgs::servoj tool_msg;
                    tool_msg.keepalive = 1;
                    tool_msg.cmd_q.resize(4);
                    for(unsigned int i=0;i<3;i++){
                        tool_msg.cmd_q[i] = tool_pos_to_slove[i];
						//std::cout<<tool_pos_to_slove[i]<<"  ";
					}
					//std::cout<<std::endl;
                    double gripper_tmp = omega_button / 0.5 * 0.78;
                    if(gripper_tmp > gripper_max)
                        gripper_tmp = gripper_max;
                    else if(gripper_tmp < gripper_min)
                        gripper_tmp = gripper_min;
                    tool_msg.cmd_q[3] = gripper_tmp;

                    pub_franka_script.publish(msg);
					if(mode==3 || mode==4)
						tool_msg.keepalive = 0;
                    pub_tool.publish(tool_msg);
                 }
             }else{
             std::cout<<"joint_distance: "<<joint_distance<<std::endl;
             std::cout<<"tool_distance: "<<tool_distance<<std::endl;
                 if(!sim){
                    franka_msgs::servoj msg;
                    msg.keepalive = 0;

                    franka_msgs::servoj tool_msg;
                    tool_msg.keepalive = 0;
                    pub_tool.publish(tool_msg);
                    pub_franka_script.publish(msg);
                 }
             }
        }else{
            if(!sim){
                franka_msgs::servoj msg;
                msg.keepalive = 0;
                franka_msgs::servoj tool_msg;
                tool_msg.keepalive = 0;
                pub_tool.publish(tool_msg);
                pub_franka_script.publish(msg);
            }
        }

        loop_rate.sleep();
    }
    std::cout<<"exit control thread"<<std::endl;
}

void franka_teleoperation::joint_position_callback(const sensor_msgs::JointState::ConstPtr& msg){
    for(unsigned int i=0;i<7;i++){
        joint_positions[i] = msg->position[i];
    }
}

void franka_teleoperation::tool_position_callback(const sensor_msgs::JointState::ConstPtr& msg){
    tool_pos[0] = msg->position[0];
    tool_pos[1] = msg->position[1];
    tool_pos[2] = msg->position[2];

    if(tool_pos[0]>tool1_max){
        ROS_ERROR("tool_joint_1_exceed_max_value");
        exit(-1);
    }else if(tool_pos[0]<tool1_min){
        ROS_ERROR("tool_joint_1_exceed_min_value");
        exit(-1);
    }

    if(tool_pos[1]>tool2_max){
        ROS_ERROR("tool_joint_2_exceed_max_value");
        exit(-1);
    }else if(tool_pos[1]<tool2_min){
        ROS_ERROR("tool_joint_2_exceed_min_value");
        exit(-1);
    }

    if(tool_pos[2]>tool3_max){
        ROS_ERROR("tool_joint_3_exceed_max_value");
        exit(-1);
    }else if(tool_pos[2]<tool3_min){
        ROS_ERROR("tool_joint_3_exceed_min_value");
        exit(-1);
    }
}

void franka_teleoperation::touch_map_callback(const robot_msgs::touch::ConstPtr& msg){
    for(unsigned int i=0;i<3;i++)
        master1_pos[i] = msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master1_rpy[i] = msg->data[i+3];
    
    touch_button = msg->button;
}

void franka_teleoperation::omega_map_callback(const robot_msgs::omega::ConstPtr& msg){
    for(unsigned int i=0;i<3;i++)
        master1_pos[i] = msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master1_rpy[i] = msg->data[i+3];
    
    omega_button = msg->button[0];
}
