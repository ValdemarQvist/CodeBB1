#include <velocity_controller.h>

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "velocity_controller");
	
	VelocityController vc;
	ros::spin();
}