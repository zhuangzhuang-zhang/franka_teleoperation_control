cmake_minimum_required(VERSION 2.8.3)
project(master_driver)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  robot_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include
  /usr/include/HD
  /usr/include/HDU
  /usr/include/eigen3
  ${PROJECT_SOURCE_DIR}/omega7_driver/include
)

add_executable(touch_driver_node src/touch_driver.cpp)
add_executable(omega_driver_node src/omega_driver.cpp)
add_executable(omega_driver_node_two src/omega_driver_two.cpp)


target_link_libraries(touch_driver_node
  ${catkin_LIBRARIES} 
  /usr/lib/libHD.so
  /usr/lib/libHDU.a
  /usr/lib/libQH.so
  /usr/lib/x86_64-linux-gnu/libstdc++.so.6
)
add_dependencies(touch_driver_node robot_msgs_gencpp)
target_link_libraries(omega_driver_node
  ${catkin_LIBRARIES} 
  ${PROJECT_SOURCE_DIR}/omega7_driver/lib/release/lin-x86_64-gcc/libdrd.so.3
  ${PROJECT_SOURCE_DIR}/omega7_driver/lib/release/lin-x86_64-gcc/libdhd.so.3
)
add_dependencies(omega_driver_node robot_msgs_gencpp)
target_link_libraries(omega_driver_node_two
  ${catkin_LIBRARIES} 
  ${PROJECT_SOURCE_DIR}/omega7_driver/lib/release/lin-x86_64-gcc/libdrd.so.3
  ${PROJECT_SOURCE_DIR}/omega7_driver/lib/release/lin-x86_64-gcc/libdhd.so.3
)
add_dependencies(omega_driver_node_two robot_msgs_gencpp)
