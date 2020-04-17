#include "ros/ros.h"
#include "libIsolationManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "isolation_manager_node");
    
    IsolationManager MyControl;

    MyControl.Execute();

    return 0;
}
