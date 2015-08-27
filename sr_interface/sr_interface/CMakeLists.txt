cmake_minimum_required(VERSION 2.8.3)
project(sr_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sr_example
  sr_grasp
  sr_multi_description
  sr_multi_moveit
  sr_robot_commander
  sr_robot_launch
)


catkin_metapackage()