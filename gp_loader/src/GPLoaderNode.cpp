#include "ros/ros.h"
#include "libGPLoaderNode.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "gp_loader_node");
    
    GPLoader MyControl;

    MyControl.Manager();

    return 0;
}
