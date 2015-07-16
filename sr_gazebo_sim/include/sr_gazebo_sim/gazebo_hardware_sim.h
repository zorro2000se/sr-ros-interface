
#ifndef SR_ROS_INTERFACE_GAZEBO_HARDWARE_SIM_H
#define SR_ROS_INTERFACE_GAZEBO_HARDWARE_SIM_H

// ros_control
//#include <control_toolbox/pid.h>
//#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/robot_hw.h>
//#include <joint_limits_interface/joint_limits.h>
//#include <joint_limits_interface/joint_limits_interface.h>
//#include <joint_limits_interface/joint_limits_rosparam.h>
//#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
//#include <gazebo/common/common.hh>
//#include <gazebo/physics/physics.hh>
//#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include "ros_ethercat_model/robot_state.hpp"

// gazebo_ros_control
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

namespace sr_gazebo_sim {

//    class SrGazeboHWSim : public gazebo_ros_control::RobotHWSim {
//    public:
//
//        virtual bool initSim(
//                const std::string &robot_namespace,
//                ros::NodeHandle model_nh,
//                gazebo::physics::ModelPtr parent_model,
//                const urdf::Model *const urdf_model,
//                std::vector <transmission_interface::TransmissionInfo> transmissions);
//
//        virtual void readSim(ros::Time time, ros::Duration period);
//
//        virtual void writeSim(ros::Time time, ros::Duration period);
//
//    };

class SrGazeboHWSim : public gazebo_ros_control::DefaultRobotHWSim {
public:

  SrGazeboHWSim();

  bool initSim(
      const std::string &robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector <transmission_interface::TransmissionInfo> transmissions);

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

private:
  ros_ethercat_model::RobotState fake_state_;

};


typedef boost::shared_ptr <SrGazeboHWSim> SrGazeboHWSimPtr;

}


#endif //SR_ROS_INTERFACE_GAZEBO_HARDWARE_SIM_H
