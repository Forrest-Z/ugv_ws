#ifndef LIB_MAPMANAGER_H
#define LIB_MAPMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>


using std::vector;
using std::cout;
using std::endl;

const int ROS_RATE_HZ   = 20;


class MapManager
{
public:
  MapManager();
  ~MapManager();

  void Manager();
  void Mission();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/
  
  /** Subscribers **/


  /** Parameters **/

  /** Flags **/

  /** Variables **/


  /** Functions **/
 

  
};


#endif
