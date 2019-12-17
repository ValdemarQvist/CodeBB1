/**
 *               battLvlIndicator
 *  Is a node that allows the user to see if the battery level
 *  by indicating  with LED on the turlebot base if the level is
 *  below 75, 50 or 25 percent of the battery charge left.
 *
 *  it read the core msg passed by the turlebot and make a desion on that
 *
 *  this is made for a fifth semester project
 *
 *  made by Grp. 19gr566 at AaU
 */


// ROS required libareis
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Led.h>



class battLvlIndicator
{
public:

battLvlIndicator()
{
	ros::Rate r(10);


	while (ros::ok()) {
		battLvlCore = nh.subscribe("/mobile_base/sensors/core", 1, &battLvlIndicator::battCallback, this);

		battLvlLed = nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
		r.sleep();
		ros::spinOnce();
	}
}


void battCallback(const kobuki_msgs::SensorState msg)
{
	// battery voltage in 0.1V (ex. 16.1V -> 161)
	int batt = msg.battery;

	kobuki_msgs::Led lMsg;
	lMsg.value = 1;
	if (batt <= 150)
		lMsg.value = 2;
	if (batt < 140)
		lMsg.value = 3;
	battLvlLed.publish(lMsg);
	ros::spinOnce();
}

~battLvlIndicator()
{
	ros::spinOnce();
	ros::shutdown();
}
private:
ros::NodeHandle nh;

ros::Publisher battLvlLed;

ros::Subscriber battLvlCore;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "battLvl2LedOutput");

	battLvlIndicator bli;

	return 0;
}
