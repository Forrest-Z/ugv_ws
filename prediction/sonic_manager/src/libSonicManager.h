#ifndef LIB_SONICMANAGER_H
#define LIB_SONICMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>


#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <boost/asio/buffer.hpp>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::to_string;

const int ROS_RATE_HZ   = 20;
const double PI = 3.14159265359;

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

using namespace boost;

class SonicManager
{
public:
  SonicManager();
  ~SonicManager();

  void Manager();
  void Mission();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/
  ros::Publisher state_pub;

  /** Subscribers **/

  /** Parameters **/
  string port_name_;
  int baud_rate_;

  serial_port_ptr port_;
  boost::system::error_code error_code_;
  boost::asio::io_service io_service_;

  /** Flags **/

  /** Variables **/
  int sonic_state_;
  /** Functions **/
  void readBaseData();
  void sendState();

  
};


#endif

