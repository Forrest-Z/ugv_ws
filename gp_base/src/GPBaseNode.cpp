#include "ros/ros.h"
#include "libGPBaseNode.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "gp_base_node");
    
    GPBase MyControl;

    MyControl.Manager();

    return 0;
}
