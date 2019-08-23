#include "ros/ros.h"
#include "libSonicManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "sonic_manager_node");
    
    SonicManager MyControl;

    MyControl.Manager();

    return 0;
}
