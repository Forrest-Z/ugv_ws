#include "ros/ros.h"
#include "libRvizManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "rviz_manager_node");
    
    RvizManager MyControl;

    MyControl.Manager();

    return 0;
}
