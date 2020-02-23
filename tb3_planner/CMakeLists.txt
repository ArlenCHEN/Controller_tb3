cmake_minimum_required(VERSION 2.8.3)
project(tb3_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  turtlebot3_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tb3_planner
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tb3_planner.cpp
# )

add_executable(tb3_planner_node src/tb3_planner_node.cpp)

target_link_libraries(tb3_planner_node
  ${catkin_LIBRARIES}
)
