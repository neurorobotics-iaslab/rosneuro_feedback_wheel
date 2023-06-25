#include <ros/ros.h>
#include "rosneuro_feedback_wheel/ControlWheel.h"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "controlwheel");

	ros::NodeHandle nh;
	rosneuro::feedback::ControlWheel wheel;

	if(wheel.configure() == false) {
		ROS_ERROR("ControlWheel configuration failed");
		ros::shutdown();
		return 0;
	}


	wheel.run();

	ros::shutdown();

	return 0;
}
