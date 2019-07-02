#include "ros/ros.h"
#include "libObstacleManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "obstacle_manager_node");
    
    ObstacleManager MyControl;

    MyControl.Manager();

    return 0;
}
