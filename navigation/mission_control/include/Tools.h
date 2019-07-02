#ifndef LIB_TOOLS_H
#define LIB_TOOLS_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>

using std::cout;
using std::endl;
using std::string;

const int ROS_RATE_HZ = 20;

class Tools
{
public:
  Tools();
  ~Tools();

  void recordLog(string input,int level);
  
};


#endif
