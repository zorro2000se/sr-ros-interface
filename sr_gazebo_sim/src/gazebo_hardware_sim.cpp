
#include "sr_gazebo_sim/gazebo_hardware_sim.h"

#include <string>

using namespace std;
using boost::ptr_unordered_map;
using namespace ros_ethercat_model;


namespace sr_gazebo_sim
{


//    bool SrGazeboHWSim::initSim(
//            const std::string& robot_namespace,
//            ros::NodeHandle model_nh,
//            gazebo::physics::ModelPtr parent_model,
//            const urdf::Model *const urdf_model,
//            std::vector<transmission_interface::TransmissionInfo> transmissions)
//    {
//        return true;
//    }
//
//    void SrGazeboHWSim::readSim(ros::Time time, ros::Duration period)
//    {
//    }
//
//    void SrGazeboHWSim::writeSim(ros::Time time, ros::Duration period)
//    {
//    }

SrGazeboHWSim::SrGazeboHWSim() : DefaultRobotHWSim(), fake_state_(NULL)
{

}


bool SrGazeboHWSim::initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions)
{
    bool result = gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model,
                                                                 transmissions);

    if (result) {

//        //        string robotParam = robot_namespace + "/robot_description";
//        //        string urdf_param_name;
//        string urdf_string;
//        // search and wait for robot_description on param server
//        //        while (urdf_string.empty())
//        //        {
//        //            ROS_INFO_STREAM(" !!! waiting for urdf: " << robotParam << " on the param server");
//        //            if (model_nh.searchParam(robotParam, urdf_param_name))
//        //            {
//        //                model_nh.getParam(urdf_param_name, urdf_string);
//        //                ROS_INFO_STREAM("found upstream");
//        //            }
//        //            else
//        //            {
//        //                model_nh.getParam(robotParam, urdf_string);
//        //                ROS_INFO_STREAM("found in node namespace");
//        //            }
//        //            ROS_INFO_STREAM(robotParam << "\n------\n" << urdf_param_name << "\n------\n" << urdf_string);
//        //            usleep(100000);
//        //        }
//        //        ROS_INFO_STREAM("gazebo controller manager got URDF from param server, parsing it...");

//        model_nh.getParam("/robot_description", urdf_string);

//        if (urdf_string.empty())
//        {
//            ROS_INFO_STREAM("URDF is empty");
//        }
//        //         else
//        //         {
//        //             ROS_INFO_STREAM("Loaded URDF " << urdf_string);
//        //         }

//        // initialize TiXmlDocument doc with a string
//        TiXmlDocument doc;
//        if (!doc.Parse(urdf_string.c_str()) && doc.Error())
//        {
//            ROS_INFO_STREAM("Could not load the gazebo controller manager plugin's configuration file: " << urdf_string);
//        }
//        else
//        {
            bool flag = false;
            while (flag)
            {
                usleep(10000);
            }

            this->fake_state_.robot_model_ = *urdf_model;

            for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it = urdf_model->joints_.begin();
                 it != urdf_model->joints_.end();
                 ++it)
            {
                // we are only loading joints that can be controlled
                if (it->second->type == urdf::Joint::PRISMATIC || it->second->type == urdf::Joint::REVOLUTE)
                    this->fake_state_.joint_states_[it->first].joint_ = it->second;
            }

//            for (TiXmlElement *xit = root->FirstChildElement("transmission");
//                 xit;
//                 xit = xit->NextSiblingElement("transmission"))
//            {
//                std::string type = xit->Attribute("type");

//                Transmission *t = transmission_loader_.createUnmanagedInstance(type);
//                if (!t || !t->initXml(xit, this))
//                    throw std::runtime_error(std::string("Failed to initialize transmission type: ") + type);
//                transmissions_.push_back(t);
//            }


            //            this->fake_state_.initXml(doc.RootElement());

            for (ptr_unordered_map<string, JointState>::iterator jit = this->fake_state_.joint_states_.begin();
                 jit != this->fake_state_.joint_states_.end();
                 ++jit)
            {
//                ROS_INFO_STREAM("Joint name in fake_state: " << jit->first);
                jit->second->calibrated_ = true;
            }

            registerInterface(&this->fake_state_);
            ROS_INFO_STREAM("registered fake state");
        }
// //    }

    return result;
}

void SrGazeboHWSim::readSim(ros::Time time, ros::Duration period)
{
    gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);


    for(unsigned int j=0; j < n_dof_; j++)
    {
        this->fake_state_.getJointState(joint_names_[j])->position_ = joint_position_[j];
        this->fake_state_.getJointState(joint_names_[j])->velocity_ = joint_velocity_[j];
        this->fake_state_.getJointState(joint_names_[j])->effort_ = joint_effort_[j];
    }
}

void SrGazeboHWSim::writeSim(ros::Time time, ros::Duration period)
{
    for(unsigned int j=0; j < n_dof_; j++)
    {
        joint_position_[j] = this->fake_state_.getJointState(joint_names_[j])->position_;
        joint_position_command_[j] = this->fake_state_.getJointState(joint_names_[j])->commanded_position_;

        joint_velocity_[j] = this->fake_state_.getJointState(joint_names_[j])->velocity_;
        joint_velocity_command_[j] = this->fake_state_.getJointState(joint_names_[j])->commanded_velocity_;

        joint_effort_[j] = this->fake_state_.getJointState(joint_names_[j])->effort_;
        joint_effort_command_ [j] = this->fake_state_.getJointState(joint_names_[j])->commanded_effort_;
    }

    gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
}


}

PLUGINLIB_EXPORT_CLASS(sr_gazebo_sim::SrGazeboHWSim, gazebo_ros_control::RobotHWSim)
//GZ_REGISTER_MODEL_PLUGIN(sr_mechanism_model::SrGazeboHWSim)
