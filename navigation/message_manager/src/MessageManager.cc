#include "ros/ros.h"
#include "libMessageManager.h"

int main(int argc, char ** argv) {

    ros::init(argc, argv, "message_manager_node");
    
    MessageManager MyControl;

    MyControl.Execute();

    return 0;
}
