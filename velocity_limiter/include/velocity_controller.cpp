#include "velocity_controller.h"

VelocityController::VelocityController() {
    sub_ = n_.subscribe("/sensorPosition", 1, &VelocityController::updateParametersCallback_, this);
    pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
}

void VelocityController::savePosition_(const geometry_msgs::Point::ConstPtr& msg) {
    position_ = *msg;
}

void VelocityController::calculateDistance_() {
    distance_ = sqrt(square(position_.x) + square(position_.y));
    ROS_INFO("Distance %f", distance_);
    if (distance_ > 3 && !std::isnan(distance_))
        pub_.publish(ac_);
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

void VelocityController::updateParametersCallback_(const geometry_msgs::Point::ConstPtr& msg) {
    savePosition_(msg);
    calculateDistance_();
    calculateVelocity_();
    sendParameter_("max_vel_x", max_velocity_);
}
