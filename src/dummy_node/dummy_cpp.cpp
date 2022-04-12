#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include<ctime>

int main(int argc, char **argv) {

	ros::init(argc, argv, "dummy_cpp");

	ros::NodeHandle n;

	double loop_rate_freq = 100.0;
	n.getParam("loop_rate", loop_rate_freq);

	ros::Publisher pub = n.advertise<std_msgs::String>("dummy_cpp_topic", loop_rate_freq);
	ros::Rate rate(loop_rate_freq);

	while (ros::ok()) {

		std_msgs::String msg;
		std::stringstream message;
		message << "Publish new message " << std::time(0);
		msg.data = message.str();

		pub.publish(msg);

		ros::spinOnce();
		rate.sleep();

	}

}
