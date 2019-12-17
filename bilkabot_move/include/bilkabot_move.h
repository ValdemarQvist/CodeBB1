#include <cstdlib>
#include <sstream>
#include <gtkmm.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#ifndef BILKABOT_MOVE_H
#define BILKABOT_MOVE_H

class Robot {
    public:
    Robot();
    ~Robot();
    void setGoal(double x, double y);

    private:
    ros::NodeHandle nh_;
    MoveBaseClient *ac_ptr_;
    move_base_msgs::MoveBaseGoal goal_;
};

class Interface : public Robot {
    private:
    void showInterface();
    void setWindowSettings();
    void setGridSettings();
    void setButtonLabels();
    void setButtonSize();
    void setButtonCallbacks();

    Gtk::Window *pWindow_;
    Gtk::ScrolledWindow scrolledWindow_;
    Gtk::Grid grid_;
    Gtk::Button button_arr_[100];
    std::ostringstream sstream_;

    Robot robot_;
    
    public:
    Interface(Gtk::Window&);
};

#endif