#include "ros/ros.h"
#include "libNaviManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "navi_manager_node");
    
    NaviManager MyControl;

    MyControl.Manager();

    return 0;
}
