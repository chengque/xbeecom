#include "ros/ros.h"
#include "std_msgs/String.h"
 #include <std_msgs/Float32MultiArray.h> 
#include <sstream>
#include "sys/time.h"
#include <math.h>
#include <stdio.h>
#include "ftditools.h"



double getTime()
{
    struct timeval t;
    gettimeofday(&t,NULL);
    double rt=t.tv_usec;
    rt=rt/1000000+t.tv_sec;
    return rt;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
	ros::Rate loop_rate(50);
	short data_out[10],data_send[4];
	int data_new_flag=0;
	int count = 0;
	double HL_Status[2];
	std_msgs::Float32MultiArray msg;
	msg.data.resize(5);
	double ltime,ntime;
	ltime=getTime();
	ntime=ltime;
	/*
	msg.layout.dim[0].size=2;
	msg.layout.dim[0].stride=2;
	msg.layout.dim[0].label="Time";
	msg.layout.dim[1].size=1;
	msg.layout.dim[2].size=1;
	*/
	openFtdi = open_ftdi(57200, "Quad_USB1", 5, 15);
	while (ros::ok()) {
		ntime=getTime();
		ltime=ntime;
		//msg.data[1]=getTime();
		//ROS_INFO("Show Data: [%s]", msg->data.c_str());
/*
		
		for(int i=0;i<8;i++)
		{
			msg.data[i]=data_out[i]/512.0f;
		}*/
		data_new_flag = read_ftdi (data_out, HL_Status);
		send_ftdi (data_send);
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
