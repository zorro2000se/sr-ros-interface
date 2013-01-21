
#include "ros/ros.h"
#include "spcu/SPCU.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spcu");
    ros::NodeHandle n; // inits this node
    spcu::SPCU s;
    s.setup();
    ros::spin();
    return 0;
}
