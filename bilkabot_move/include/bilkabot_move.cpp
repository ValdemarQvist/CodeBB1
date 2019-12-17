#include "bilkabot_move.h"

Robot::Robot() {
    ac_ptr_ = new MoveBaseClient("move_base", true);
}

Robot::~Robot() {
    delete ac_ptr_;
}

void Robot::setGoal(double x, double y) {
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();
    goal_.target_pose.pose.position.x = x;
    goal_.target_pose.pose.position.y = y;
    goal_.target_pose.pose.orientation.w = 1.0;
    ac_ptr_->sendGoal(goal_);
}

Interface::Interface(Gtk::Window& w) : pWindow_(&w) {
    setWindowSettings();
    setGridSettings();
    setButtonLabels();
    setButtonCallbacks();
    setButtonSize();
    showInterface();
}

void Interface::setWindowSettings() {
  pWindow_->set_default_size(800, 400);
  pWindow_->set_title("Bilkabot");
  pWindow_->set_border_width(10);
  pWindow_->add(scrolledWindow_);
  scrolledWindow_.add(grid_);
  scrolledWindow_.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
}


void Interface::setGridSettings() {
    int j = 0;
    for (int i = 0; i < 100; i++) {
        if (!(i % 10)) {
            j++;
        }
        grid_.attach(button_arr_[i], i%10, j, 1, 1);
    }
}


void Interface::setButtonLabels() {
    for (int i = 0; i < 100; i++) {
        sstream_.str("");
        sstream_ << i;
        button_arr_[i].set_label("Section " + sstream_.str());
    }
}


void Interface::setButtonSize() {
    for (int i = 0; i < 100; i++)
        button_arr_[i].set_size_request(50, 50);
}


void Interface::setButtonCallbacks() {
    for (int i = 0; i < 100; i++)
        button_arr_[i].signal_clicked().connect(sigc::bind<int>(sigc::mem_fun(*this,&Interface::setGoal), i, i));
}


void Interface::showInterface() {
  pWindow_->show_all_children();
}