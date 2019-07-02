#include "ros/ros.h"
#include "libJCBaseNode.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "jc_base_node");
    
    JCBase MyControl;

    MyControl.Manager();

    return 0;
}
