#!/bin/bash

# echo "smj"|sudo -S /home/smj/c++test/build/m_test11

#source /opt/ros/kinetic/setup.bash
#roscore


source /home/master/franka_final_ws/devel/setup.bash
roslaunch master_driver omega_touch_robot.launch&

sleep 2
source /home/master/franka_final_ws/devel/setup.bash
rosrun servo_control_class franka1_omega1_node&

sleep 2
source /home/master/franka_final_ws/devel/setup.bash
rosrun servo_control_class franka2_omega2_node&
