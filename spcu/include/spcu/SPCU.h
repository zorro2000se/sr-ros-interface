#pragma once

#include "ros/ros.h"
#include <cereal_port/CerealPort.h>
#include "spcu/ActuatorCommand.h"
#include "spcu/ControllerState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include "spcu/SetController.h"
#include "spcu/TargetList.h"

using namespace std;
using namespace ros;

namespace spcu {

    class SPCU {
    public:
        SPCU();
        ~SPCU();

        /// Setup topics and then call connect()
        void setup();
        void connect();

    private:
        NodeHandle nh, n_home;
        string port_name;
        int baud_rate;
        double publish_rate;
        cereal::CerealPort port;
        unsigned int sensorValues[8];
        bool connected;

        bool isConnected();

        void sendCmd(char* cmd, int cmd_size);

        Subscriber valveSetSub[8];
        void valveSet(int valve_num, const std_msgs::Bool::ConstPtr& msg);
        void valve0Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(0, msg); };
        void valve1Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(1, msg); };
        void valve2Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(2, msg); };
        void valve3Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(3, msg); };
        void valve4Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(4, msg); };
        void valve5Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(5, msg); };
        void valve6Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(6, msg); };
        void valve7Set(const std_msgs::Bool::ConstPtr& msg) { valveSet(7, msg); };

        Subscriber valvePulseSub[8];
        void valvePulse(int valve_num, const int& millis);
        void valve0Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(0, msg->data); }
        void valve1Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(1, msg->data); }
        void valve2Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(2, msg->data); }
        void valve3Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(3, msg->data); }
        void valve4Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(4, msg->data); }
        void valve5Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(5, msg->data); }
        void valve6Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(6, msg->data); }
        void valve7Pulse(const std_msgs::Int16::ConstPtr& msg) { valvePulse(7, msg->data); }

        Subscriber setTargetsSub;
        void setTargets(const TargetList& msg);
        Subscriber valveTargetSub[8];
        void valveTarget(int valve_num, const int& millis);
        void valve0Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(0, msg->data); }
        void valve1Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(1, msg->data); }
        void valve2Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(2, msg->data); }
        void valve3Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(3, msg->data); }
        void valve4Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(4, msg->data); }
        void valve5Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(5, msg->data); }
        void valve6Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(6, msg->data); }
        void valve7Target(const std_msgs::UInt16::ConstPtr& msg) { valveTarget(7, msg->data); }

        Subscriber actuatorCommandSub[4];
        void actuatorCommand(int act_num, const spcu::ActuatorCommand::ConstPtr& cmd);
        void actuator0Command(const spcu::ActuatorCommand::ConstPtr & cmd) { actuatorCommand(0, cmd); }
        void actuator1Command(const spcu::ActuatorCommand::ConstPtr & cmd) { actuatorCommand(2, cmd); }
        void actuator2Command(const spcu::ActuatorCommand::ConstPtr & cmd) { actuatorCommand(4, cmd); }
        void actuator3Command(const spcu::ActuatorCommand::ConstPtr & cmd) { actuatorCommand(6, cmd); }

        Timer updateTimer;
        Publisher sensorPub[8];
        Publisher controllerPub[8];
        Publisher targetsPub;
        void update(const TimerEvent&);
        void update();

        ServiceServer controllerSrv;
        bool setController(SetController::Request &req, SetController::Response &res);
    };

} //spcu::
