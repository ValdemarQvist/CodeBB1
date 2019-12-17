#include "bilkabot_move.h"


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "bilkabot_move");

    Gtk::Main kit(argc, argv);
    Gtk::Window window;
    
    Interface ui(window);

    Gtk::Main::run(window);
}