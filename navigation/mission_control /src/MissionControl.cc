#include "ros/ros.h"
#include "libMissionControl.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "mission_control_node");

    int a = 1;
    int b = 2;
    std::cout << a+b << endl;
    
    MissionControl MyControl;

    MyControl.Execute();

    return 0;
}
