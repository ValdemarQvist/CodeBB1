#ifndef PEOPLE_LEGS_H
#define PEOPLE_LEGS_H

#include <algorithm>
#include <vector>
#include <cmath>
#include <boost/bind.hpp>
#include "ros/ros.h"
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

#define square(x) pow(x, 2)

class PeopleLegs {
    private:
        ros::NodeHandle n_;
        ros::Subscriber detector_sub1_, detector_sub2_;
        ros::Publisher people_pub_;
        std::vector<people_msgs::PositionMeasurement> people_, people1_, people2_;
        std::vector<double> distance1_, distance2_;
	    people_msgs::PositionMeasurementArray people_msg_;

        enum Tracker{
            TRACKER_1 = 1,
            TRACKER_2 = 2
        } t_;

    public:
        PeopleLegs();

    private:
        bool isSamePerson_();
        void updateParamCallback_(const people_msgs::PositionMeasurementArray::ConstPtr&, PeopleLegs::Tracker);
};

#endif
