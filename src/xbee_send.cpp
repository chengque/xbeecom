#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h> 
#include <sstream>
#include "sys/time.h"
#include <math.h>
#include <stdio.h>
#include "ftditools.h"


void poseHandler(geometry_msgs::PoseStamped msg)
{
    short data[1];
    int secs[2];
    data[0]=msg.pose.position.x;
    data[1]=msg.pose.position.y;
    data[2]=msg.pose.position.z;
    data[3]=msg.pose.orientation.x;
    data[4]=msg.pose.orientation.y;
    data[5]=msg.pose.orientation.z;
    data[6]=msg.pose.orientation.w;
    secs[0]=msg.header.stamp.sec;
    secs[1]=msg.header.stamp.nsec;
    memcpy(&data[7],secs,sizeof(secs));
	if(openFtdi<1)
	{
		openFtdi = open_ftdi(57600, "Aero0_Groud", 5, 15);
	}
	send_ftdi(data);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "xbee sender");
	ros::NodeHandle n;
	openFtdi = open_ftdi(57600, "Aero0_Groud", 5, 15);
	ROS_INFO("Open Ftdi: [%d]", openFtdi);
	ros::Subscriber sub = n.subscribe("/mocap/pose", 10, poseHandler);
	ros::spin();
	return 0;
}
