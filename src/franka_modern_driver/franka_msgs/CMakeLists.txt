cmake_minimum_required(VERSION 2.8.3)
project(franka_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES Errors.msg FrankaState.msg servoj.msg)
add_service_files(FILES error_recovery.srv)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
