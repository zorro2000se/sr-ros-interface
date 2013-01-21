#include "spcu/SPCU.h"
#include <sstream>

namespace spcu {

using boost::lexical_cast;

// Get (unsigned) int value from two bytes with most significant bit first.
// This is how the SPCU often returns data.
unsigned int bytes2int(const char& b1, const char& b2) {
    unsigned int value = 0;
    value = (value << 8) + (int)(unsigned char)b1;
    value = (value << 8) + (int)(unsigned char)b2;
    return value;
}

void int2bytes(const int& value, char* out) {
    out[0] = (unsigned char)(value >> 8);
    out[1] = (unsigned char)value;
}

string bytes2hexstr(char* bytes, int len) {
    stringstream ss;
    ss << hex << setfill('0');
    for (int i=0; i<len; ++i)
        ss << setw(2) << int(bytes[i]) << " ";
    return ss.str();
}

SPCU::SPCU() : n_home("~"), connected(false) {
    n_home.param<string>("port_name", port_name, "/dev/ttyS0");
    n_home.param("baud_rate", baud_rate, 19200);
    n_home.param("publish_rate", publish_rate, 0.1);

    for (int i=1; i<8; ++i)
        sensorValues[i] = 0;
}

SPCU::~SPCU() {
    if (port.portOpen())
        port.close();
}

bool SPCU::isConnected() {
	return connected;
}

void SPCU::setup() {
    valveSetSub[0] = n_home.subscribe("valve0/set", 1, &SPCU::valve0Set, this);
    valveSetSub[1] = n_home.subscribe("valve1/set", 1, &SPCU::valve1Set, this);
    valveSetSub[2] = n_home.subscribe("valve2/set", 1, &SPCU::valve2Set, this);
    valveSetSub[3] = n_home.subscribe("valve3/set", 1, &SPCU::valve3Set, this);
    valveSetSub[4] = n_home.subscribe("valve4/set", 1, &SPCU::valve4Set, this);
    valveSetSub[5] = n_home.subscribe("valve5/set", 1, &SPCU::valve5Set, this);
    valveSetSub[6] = n_home.subscribe("valve6/set", 1, &SPCU::valve6Set, this);
    valveSetSub[7] = n_home.subscribe("valve7/set", 1, &SPCU::valve7Set, this);

    valvePulseSub[0] = n_home.subscribe("valve0/pulse", 1, &SPCU::valve0Pulse, this);
    valvePulseSub[1] = n_home.subscribe("valve1/pulse", 1, &SPCU::valve1Pulse, this);
    valvePulseSub[2] = n_home.subscribe("valve2/pulse", 1, &SPCU::valve2Pulse, this);
    valvePulseSub[3] = n_home.subscribe("valve3/pulse", 1, &SPCU::valve3Pulse, this);
    valvePulseSub[4] = n_home.subscribe("valve4/pulse", 1, &SPCU::valve4Pulse, this);
    valvePulseSub[5] = n_home.subscribe("valve5/pulse", 1, &SPCU::valve5Pulse, this);
    valvePulseSub[6] = n_home.subscribe("valve6/pulse", 1, &SPCU::valve6Pulse, this);
    valvePulseSub[7] = n_home.subscribe("valve7/pulse", 1, &SPCU::valve7Pulse, this);

    setTargetsSub = n_home.subscribe("targets/set", 1, &SPCU::setTargets, this);
    valveTargetSub[0] = n_home.subscribe("valve0/target", 1, &SPCU::valve0Target, this);
    valveTargetSub[1] = n_home.subscribe("valve1/target", 1, &SPCU::valve1Target, this);
    valveTargetSub[2] = n_home.subscribe("valve2/target", 1, &SPCU::valve2Target, this);
    valveTargetSub[3] = n_home.subscribe("valve3/target", 1, &SPCU::valve3Target, this);
    valveTargetSub[4] = n_home.subscribe("valve4/target", 1, &SPCU::valve4Target, this);
    valveTargetSub[5] = n_home.subscribe("valve5/target", 1, &SPCU::valve5Target, this);
    valveTargetSub[6] = n_home.subscribe("valve6/target", 1, &SPCU::valve6Target, this);
    valveTargetSub[7] = n_home.subscribe("valve7/target", 1, &SPCU::valve7Target, this);

    actuatorCommandSub[0] = n_home.subscribe("actuator0/command", 1, &SPCU::actuator0Command, this);
    actuatorCommandSub[1] = n_home.subscribe("actuator1/command", 1, &SPCU::actuator1Command, this);
    actuatorCommandSub[2] = n_home.subscribe("actuator2/command", 1, &SPCU::actuator2Command, this);
    actuatorCommandSub[3] = n_home.subscribe("actuator3/command", 1, &SPCU::actuator3Command, this);

    targetsPub = n_home.advertise<spcu::TargetList>("targets", 5);
    for (int i=0; i<8; ++i) {
        string num = lexical_cast<string>(i);
        sensorPub[i] = n_home.advertise<std_msgs::UInt16>("sensor"+num, 5);
        controllerPub[i] = n_home.advertise<spcu::ControllerState>(
                "valve"+num+"/controller" , 5);
    }

    controllerSrv = n_home.advertiseService("set_controller", &SPCU::setController, this);

    if (publish_rate > 0.0) 
        updateTimer = n_home.createTimer(Duration(publish_rate), &SPCU::update, this);

    connect();
}

void SPCU::connect() {
    string version;
    try {
        port.open(port_name.c_str(), baud_rate);
        ROS_INFO_STREAM("Connected to " << port_name << " : " << port.portOpen());
        port.flush();
        port.write("?");
        port.readLine(&version, 1000); // Timeout in millis
        port.flush();
    }
    //catch(cereal::TimeoutException) {
    catch(cereal::Exception& err) {
        ROS_ERROR_STREAM("Cereal error on " << port_name << " : " << err.what());
        return;
    }
    ROS_INFO_STREAM("Version: " << version);
    connected = true;
}

// Send single command where we are not going top read the output
void SPCU::sendCmd(char* cmd, int cmd_size) {
    //ROS_INFO_STREAM("Cmd: " << bytes2hexstr(cmd, cmd_size));
	if (!isConnected()) return;

    try {
        port.flush();
        port.write(cmd, cmd_size);
        
        // Read the response, cmd echo + checksum byte
        char dummy[cmd_size+1];
        port.readBytes(dummy, cmd_size+1, 1000);
        //ROS_INFO_STREAM("Dummy: " << bytes2hexstr(dummy, cmd_size+1));
    }
    catch(cereal::Exception& err) {
        ROS_ERROR_STREAM("Cereal error in sendCmd on " << port_name << " : " << err.what());
        return;
    }
}

void SPCU::valveSet(int valve_num, const std_msgs::Bool::ConstPtr& msg) {
    char cmd[] = { 0x02, valve_num, msg->data, 0xFF };
    sendCmd(cmd, 4);
}

void SPCU::valvePulse(int valve_num, const int& millis) {
    char cmd[] = { 0x01, valve_num, millis, 0xFF };
    sendCmd(cmd, 4);
}

void SPCU::valveTarget(int valve_num, const int& target) {
    char out[2];
    int2bytes(target, out);
    char cmd[] = { 0x06, valve_num, out[0], out[1], 0xFF };
    sendCmd(cmd, 5);
}

//act_num is the number of the first of the two pumps for a port. ie 0,2,4,6
void SPCU::actuatorCommand(int act_num, const spcu::ActuatorCommand::ConstPtr & msg) {
    int pump_cmd = msg->command;
    int pulse = msg->pulse_time;

    if (!(pump_cmd == msg->PUMP_COMMAND_HOLD
            || pump_cmd == msg->PUMP_COMMAND_FILL
            || pump_cmd == msg->PUMP_COMMAND_EMPTY)) {
        ROS_ERROR_STREAM("Unknown command: " << pump_cmd);
        return;
    }

    if (pulse > 0) {
        int v;
        if (pump_cmd == msg->PUMP_COMMAND_FILL)
            v = act_num;
        else if (pump_cmd == msg->PUMP_COMMAND_EMPTY)
            v = act_num + 1;
        else
            return; // pulse hold makes no sense
        char cmd[] = { 0x01, v, pulse, 0xFF };
        //port.write(cmd, 4);
        sendCmd(cmd, 4);
        return;
    }

    // No pulse so set state
    int v1 = 0, v2 = 0; // hold
    if (pump_cmd == msg->PUMP_COMMAND_FILL)
        v1 = 1;
    else if (pump_cmd == msg->PUMP_COMMAND_EMPTY)
        v2 = 1;
    char cmd[] = { 0x02, act_num, v1, act_num+1, v2, 0xFF };
    //port.write(cmd, 6);
    sendCmd(cmd, 6);
}

void SPCU::setTargets(const TargetList& msg) {
    int num_targets = msg.valve.size();
    if (num_targets > 8) num_targets = 8;
    int cmd_size = (num_targets*3)+2;
    char cmd[cmd_size];
    cmd[0] = 0x06;
    char *ptr = cmd+1;
    for (int i=0; i<num_targets; ++i) {
        *ptr++ = msg.valve[i];
        int2bytes(msg.target[i], ptr++);
        ptr++; // We added 2 bytes
    }
    *ptr++ = 0xFF;

    sendCmd(cmd, cmd_size);
}

void SPCU::update(const TimerEvent& te) {
    update();
}

void SPCU::update() {
	if (!isConnected()) return;

    char cmd[] = { 0x08 };
    char data[89];
    try {
        port.flush(); // important to get clean read of right data
        port.write(cmd,1);
        port.readBytes(data, 88, 1000); // timeout millis
    }
    catch(cereal::Exception& err) {
        ROS_ERROR_STREAM("Cereal error in update on " << port_name << " : " << err.what());
        return;
    }

    char *ptr;

    // Sensor, target values
    ptr = data + 3;
    TargetList target_msg;
    for (int n=0; n<8; ++n) {
        char b1 = *ptr++;
        char b2 = *ptr++;
        int sensor = bytes2int(b1, b2);
        b1 = *ptr++;
        b2 = *ptr++;
        int target = bytes2int(b1, b2);

        std_msgs::UInt16 msg;
        msg.data = sensor;
        sensorPub[n].publish(msg);

        target_msg.valve.push_back(n);
        target_msg.target.push_back(target);
    }
    targetsPub.publish(target_msg);

    // Controllers
    ptr = data + 48;
    for (int n=0; n<8; ++n) {
        int sensor = (int)(unsigned char)(*ptr++);
        int target = (int)(unsigned char)(*ptr++);
        int p = ((int)(unsigned char)(*ptr++))-127;
        int i = ((int)(unsigned char)(*ptr++))-127;
        int d = ((int)(unsigned char)(*ptr++))-127;
        spcu::ControllerState msg;
        if (sensor == 255 && target == 255) {
            msg.active = false;
            msg.sensor = 0;
            msg.target = 0;
            msg.p = 0;
            msg.i = 0;
            msg.d = 0;
        }
        else {
            msg.active = true;
            msg.sensor = sensor;
            msg.target = target;
            msg.p = p;
            msg.i = i;
            msg.d = d;
        }
        controllerPub[n].publish(msg);
    }
}

bool SPCU::setController(SetController::Request &req, SetController::Response &res) {
    if (!req.active) {
        char cmd[] = { 0x05, req.valve, 0xFF };
        sendCmd(cmd, 3);
        return true;
    }

    char sensor_target = req.sensor;
    if (req.target < 8) {
        sensor_target |= 1 << 3;
    }
    sensor_target += ((req.target & 8) << 4);
    char cmd[] = { 0x04, req.valve, sensor_target, req.p+127, req.i+127, req.d+127 };
	sendCmd(cmd, 6);
    return true;
}

} //spcu::
