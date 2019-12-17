#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#define square(x) pow(x, 2)

class VelocityController {
    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        actionlib_msgs::GoalID ac_;
        geometry_msgs::Point position_;
        std::ostringstream sstream_;
        double distance_, max_velocity_, min_velocity_;
        std::vector<people_msgs::PositionMeasurement> people_;

    public:
        VelocityController();

    private:
        //void updateParametersCallback_(const geometry_msgs::Point::ConstPtr&);
        //void savePosition_(const geometry_msgs::Point::ConstPtr&);
        void updateParametersCallback_(const people_msgs::PositionMeasurementArray::ConstPtr&);
        void savePosition_(const people_msgs::PositionMeasurementArray::ConstPtr&);
        void sendParameter_(const std::string &, double);
        void calculateDistance_();
        void calculateVelocity_();
};

#endif
