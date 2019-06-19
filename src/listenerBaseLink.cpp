#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>

void callback(const geometry_msgs::PointStamped msg){
	std::cout << "The position of the detected object relative to the base link reference frame is:" << std::endl;
	std::cout << msg.point ;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "listenerBaseLink");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("base_link_pos", 1000, callback);

	ros::spin();
}