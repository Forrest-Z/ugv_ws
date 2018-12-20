#include "ros/ros.h"
#include "libMapManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "map_manager_node");
    
    MapManager MyControl;

    MyControl.Manager();

    return 0;
}
