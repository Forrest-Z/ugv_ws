#include "ros/ros.h"
#include "libLogManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "log_manager_node");
    
    LogManager MyControl;

    // MyControl.Execute();

    ros::spin();

    return 0;
}
