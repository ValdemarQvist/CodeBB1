#include "velocity_controller.h"

VelocityController::VelocityController() {
    sub_ = n_.subscribe("/leg_detector", 1, &VelocityController::updateParametersCallback_, this);
    pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
}

void VelocityController::savePosition_(const people_msgs::PositionMeasurementArray::ConstPtr& msg) {
    people_ = msg->people;
}

void VelocityController::calculateDistance_() {
    for (const people_msgs::PositionMeasurement p : people_) {
            distance_ = sqrt(square(p.pos.x) + square(p.pos.y));
    }
    ROS_INFO("Distance %f", distance_);
    if (distance_ > 3 && !std::isnan(distance_)) {
        std::string cmd = "rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS \"{'max_vel_x':0, 'min_vel_x':0, 'max_vel_theta':0, 'min_vel_theta':0, 'min_in_place_vel_theta':0}\"";
        system(cmd.c_str());
    }
}

void VelocityController::calculateVelocity_() {
   if (!std::isnan(distance_)) {
       max_velocity_ =  -(0.38*distance_) + 1.16;
       ROS_INFO("Velocity %f", max_velocity_);
       min_velocity_ = max_velocity_*0.8;
    }
}

void VelocityController::sendParameter_(const std::string &s, double value) {
    sstream_.str("");
    sstream_ << value;
    std::string cmd = "rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS " + s + " " + sstream_.str();
    system(cmd.c_str());
}

void VelocityController::updateParametersCallback_(const people_msgs::PositionMeasurementArray::ConstPtr& msg) {
    savePosition_(msg);
    calculateDistance_();
    calculateVelocity_();
    sendParameter_("max_vel_x", max_velocity_);
}
