#include "libMapManager.h"

MapManager::MapManager():pn("~")
{

  sleep(1);
}


MapManager::~MapManager(){
}

void MapManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void MapManager::Mission() {

}

