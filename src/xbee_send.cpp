#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h> 
#include "geometry_msgs/Vector3.h"
#include <sstream>
#include "sys/time.h"
#include <math.h>
#include <stdio.h>
#include "ftditools.h"

geometry_msgs::Vector3 posref;
int initsecs;


void poseHandler(geometry_msgs::PoseStamped msg)
{
    short data[14];
    int secs[2];
    data[0]=(short)(msg.pose.position.x*1000);
    data[1]=(short)(msg.pose.position.y*1000);
    data[2]=(short)(msg.pose.position.z*1000);
    data[3]=(short)(msg.pose.orientation.x*10000);
    data[4]=(short)(msg.pose.orientation.y*10000);
    data[5]=(short)(msg.pose.orientation.z*10000);
    data[6]=(short)(msg.pose.orientation.w*10000);
    data[7]=(short)(posref.x*1000);
    data[8]=(short)(posref.y*1000);
    data[9]=(short)(posref.z*1000);
    secs[0]=msg.header.stamp.sec;
    secs[1]=msg.header.stamp.nsec;
    std::cout<<secs[0]<<":"<<secs[1]<<":"<<msg.pose.position.x<<":"<<msg.pose.position.y<<":"<<msg.pose.position.z<<":"<<msg.pose.orientation.x<<":"<<msg.pose.orientation.y<<":"<<msg.pose.orientation.z<<":"<<msg.pose.orientation.w<<std::endl;
    memcpy(&data[10],secs,sizeof(secs));
	if(openFtdi<1)
	{
		openFtdi = open_ftdi(57600, "Aero0_Ground", 5, 15);
	}
	send_ftdi(data);
}

void poserefHandler(geometry_msgs::Vector3 msg)
{
	posref.x=msg.x;
	posref.y=msg.y;
	posref.z=msg.z;
	//std::cout<<"--"<<msg.y<<std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "xbee_sender");
	ros::NodeHandle n;
	openFtdi = open_ftdi(57600, "Aero0_Ground", 5, 15);
	ROS_INFO("Open Ftdi: [%d]", openFtdi);
	posref.z=-1;
	posref.y=0;
	posref.x=0;

	initsecs = (int)(ros::Time::now().toSec());
	ros::Subscriber sub = n.subscribe("/mocap/pose", 10, poseHandler);
	ros::Subscriber subref = n.subscribe("/cmd/posref", 10, poserefHandler);
	ros::spin();
	return 0;
}
