cmake_minimum_required(VERSION 2.8.3)
project(irb6640_moveit_config)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(irb6640_moveit_config src/vacuum_gripper_ctrl.cpp)
target_link_libraries(irb6640_moveit_config ${catkin_LIBRARIES})

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
