#include "ros/ros.h"
#include "libMissionControl.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "mission_control_node");
    
    MissionControl MyControl;

    MyControl.Execute();

    return 0;
}
