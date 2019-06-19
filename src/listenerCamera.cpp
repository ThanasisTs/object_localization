#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
std::string camera_frame;
class Listener{
public:
	tf::TransformListener listener;
	Listener();
	void lisCallback (const geometry_msgs::PointStamped msg);
};

Listener::Listener(){
	tf::TransformListener listener();
}

void Listener::lisCallback(const geometry_msgs::PointStamped msg){
	try{
		std::cout << msg.header.frame_id << std::endl;
		geometry_msgs::PointStamped base_point;
		base_point.header.frame_id = "base_link";
		listener.waitForTransform ("base_link", camera_frame, msg.header.stamp, ros::Duration(5.0));

		//listener.waitForTransform ("base_link", "zed_left_camera_frame", ros::Time::now(), ros::Duration(5.0));
		listener.transformPoint("base_link", msg, base_point);
		ROS_INFO("Camera Frame: (%.8f, %.8f. %.8f) -----> Base Link Frame: (%.8f, %.8f, %.8f) at time %.2f",
			msg.point.x, msg.point.y, msg.point.z, 
			base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
		pub.publish(base_point);
	}
	catch(tf::TransformException& ex){
		std::cout << "Received an exception trying to transform a point from " << camera_frame << "to base_link: " << ex.what();
		ROS_ERROR("Received an exception trying to transform a point from \"zed_left_camera_frame\" to \"base_link\": %s", ex.what());
	}
}


int main(int argc, char** argv){

	ros::init(argc, argv, "listenerCamera");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::PointStamped>("base_link_pos", 1000);
	n.param("/object_localization/camera_frame", camera_frame, std::string("zed_left_camera_frame"));
	Listener ln;
	ros::Subscriber sub = n.subscribe<geometry_msgs::PointStamped>("camera_pos", 100, &Listener::lisCallback, &ln);

	ros::spin();
}