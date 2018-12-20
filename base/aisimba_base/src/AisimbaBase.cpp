#include "ros/ros.h"
#include "libAisimbaBase.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "aisimba_base_node");
    
    AisimbaBase MyControl;

    MyControl.Manager();

    return 0;
}
