cmake_minimum_required(VERSION 2.8.3)
project(tb3_control)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  uav_utils
  turtlebot3_msgs
  nav_msgs
  )

find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
  include/control
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node 
src/control.cpp
src/pid.cpp
src/pid_param.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )