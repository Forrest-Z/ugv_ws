#include "ros/ros.h"
#include "libSpeedManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "speed_manager_node");
    
    SpeedManager MyControl;

    MyControl.Manager();

    return 0;
}
