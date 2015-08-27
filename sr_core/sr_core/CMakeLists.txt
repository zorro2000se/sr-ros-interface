cmake_minimum_required(VERSION 2.8.3)
project(sr_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sr_gazebo_sim
  sr_hand
  sr_hardware_interface
  sr_mechanism_controllers
  sr_mechanism_model
  sr_self_test
  sr_tactile_sensors
  sr_utilities
)

catkin_metapackage()