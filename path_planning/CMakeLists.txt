cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_ros_visualization
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(path_planning src/path_planning.cpp)
target_link_libraries(path_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})

