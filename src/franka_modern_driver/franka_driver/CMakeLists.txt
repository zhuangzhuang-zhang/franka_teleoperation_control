cmake_minimum_required(VERSION 2.8.3)
project(franka_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


#find_package(Franka REQUIRED)
#message(STATUS "Franka includePath: ${Franka_INCLUDE_DIRS}")
#message(STATUS "Franka libPath: ${Franka_LIBRARIES}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  franka_msgs
  actionlib
  actionlib_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Franka_INCLUDE_DIRS}
  /usr/include/eigen3
)


#add_executable(franka_modern_driver src/main.cpp src/franka_modern_driver.cpp)
#target_link_libraries(franka_modern_driver ${catkin_LIBRARIES} ${Franka_LIBRARIES})

#add_executable(franka1_modern_driver src/main_franka1.cpp src/franka_modern_driver_v2.cpp)
#target_link_libraries(franka1_modern_driver ${catkin_LIBRARIES} ${Franka_LIBRARIES})

#add_executable(franka2_modern_driver src/main_franka2.cpp src/franka_modern_driver_v2.cpp)
#target_link_libraries(franka2_modern_driver ${catkin_LIBRARIES} ${Franka_LIBRARIES})

#add_executable(joint_pub_node src/joint_pub.cpp)
#target_link_libraries(joint_pub_node ${catkin_LIBRARIES})

#add_executable(franka_modern_driver_two_arms src/main_two_arms.cpp src/franka_modern_driver_two_arms.cpp)
#target_link_libraries(franka_modern_driver_two_arms ${catkin_LIBRARIES} /home/smj/franka/libfranka/build/libfranka.so)
