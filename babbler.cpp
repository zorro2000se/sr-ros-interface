/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/* Initial WIN32 port by Hozefa Indorewala, Robotics Equipment Corporation GmbH, www.servicerobotics.eu */

#define DLL_EXPORT
#include "dllheader.h"
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/network.h"
//messages
#include <sr_hand/joints_data.h>
#include <sr_hand/joint.h>
#include <sr_hand/sendupdate.h>

#include <string>
#include "hash_map"

ros::Publisher pub;
ros::Subscriber sub;
//ros::NodeHandle n;

std::string received_name = " ";
float position = 3; 
unsigned short index_msg = 0;
int msg_length = 0;
//float positions[28];
sr_hand::joints_dataConstPtr newmsg;
//sr_hand::sendupdate sendmsg[10];
int sent = 0;
std::string masteruri1;
std::string localip1;
char *argv2;

sr_hand::sendupdate msg;
std::vector<sr_hand::joint> jointVector;
sr_hand::joint joint;


void callback(const sr_hand::joints_dataConstPtr& msg)
{
	newmsg = msg;
}

EXPORT int ROSSetup (char * argv, std::string masteruri, std::string localip) {

	//set master URI
	ros::master::setURI(masteruri);
	//ros::master::setURI("http://192.168.1.108:11311/");

	//set ros ip
	ros::network::setHost(localip);
	//ros::network::setHost("192.168.1.102");

	masteruri1 = masteruri;
	localip1 = 	localip;
	argv2 = argv;

	int argc = 1;
	char *argv1[1];
	argv1[0] = argv;

	ros::init(argc, argv1, "babbler");
	ros::NodeHandle n;

	pub = n.advertise<sr_hand::sendupdate>("/srh/sendupdate", 1000);
	sub = n.subscribe("/srh/shadowhand_data", 1000, callback);

	ros::Rate r(100);
	ros::spinOnce();

	r.sleep();
	r.sleep();
	r.sleep();
	r.sleep();
	r.sleep();
	r.sleep();
	r.sleep();

	while(n.ok() == 0)
	{
		r.sleep();
	}

	return 66;
}

EXPORT void ROSSend (std::string joint_name[20], float angle[20]) {

	//if(ros::ok())
	//{

	//sr_hand::sendupdate msg;
	//std::vector<sr_hand::joint> jointVector;
	//ros::Rate ra(1000);
	//sr_hand::joint joint;


	//fill the message
	for(int i = 0; i < 20; i++)
	{
	if(joint_name[i] == "0")
	{
		break;
	}
	else
	{
	joint.joint_name = joint_name[i];
	joint.joint_target = angle[i];
	jointVector.push_back(joint);

	}
	}
	msg.sendupdate_length = jointVector.size();
	msg.sendupdate_list = jointVector;
	pub.publish(msg);
	jointVector.clear();
	//pub.publish(msg);
	//ros::spinOnce();
	//ros::spinOnce();
	//ra.sleep();
	//return "SENT";//joint_name[i];
	//}
	return;
}

extern EXPORT float ROSRec (std::string joint_name){

	//float steve[28];
	//received_name = joint_name;
	ros::spinOnce();
	msg_length = newmsg->joints_list_length;
	
	for(index_msg=0; index_msg < msg_length; ++index_msg)
	{
		//steve[index_msg] = newmsg->joints_list[index_msg].joint_position;
		//get the sensor name
		//std::string sensor_name = msg->joints_list[index_msg].joint_name;
		if(newmsg->joints_list[index_msg].joint_name == joint_name)
		{
			position = newmsg->joints_list[index_msg].joint_position;
		}
	}
	
	//hash_map<std::string, float> hashh;
	return position;

	//return msg_length;
}

extern EXPORT bool ROSQuit (){
	ros::shutdown();
	return 1;
	

}