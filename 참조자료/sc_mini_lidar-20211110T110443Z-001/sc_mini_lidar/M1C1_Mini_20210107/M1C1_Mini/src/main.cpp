#include <stdio.h>
#include "node_lidar.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


int main(int argc, char **argv)
{
	
	node_lidar_t node_lidar{};

	ros::init(argc, argv, "M1CT_Mini");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	priv_nh.param("baud_rate", node_lidar.baudrate, 115200);
	priv_nh.param("frame_id", node_lidar.frame_id, std::string("base_link"));
	priv_nh.param("port", node_lidar.port, std::string("/dev/sc_mini"));
	node_lidar.laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
	
	printf("node_start\n");
	node_start(node_lidar);
}                                                                                                           