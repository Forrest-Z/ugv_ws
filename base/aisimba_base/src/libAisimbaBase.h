#ifndef LIB_AISIMBABASE_H
#define LIB_AISIMBABASE_H

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

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

using std::cout;
using std::endl;
using std::string;

const int ROS_RATE_HZ   = 20;

const uint8_t FORWARD = 0x03;
const uint8_t FORWARD_LEFT = 0x04;
const uint8_t FORWARD_RIGHT = 0x05;
const uint8_t STOP = 0x06;
const uint8_t BACKWARD = 0x07;
const uint8_t BACKWARD_LEFT = 0x08;
const uint8_t BACKWARD_RIGHT = 0x09;

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


class AisimbaBase
{
public:
  AisimbaBase();
  ~AisimbaBase();

  void Manager();
  void Mission();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/

  /** Subscribers **/
  ros::Subscriber cmd_sub;

  /** Parameters **/
  string port_name_;
  int baud_rate_;
  int control_rate_;


  ros::Timer send_speed_timer;
  serial_port_ptr port_;
  boost::system::error_code ec_;
  boost::asio::io_service io_service_;

  /** Flags **/


  /** Variables **/
  geometry_msgs::Twist cmd_vel_;

  /** Functions **/
  void sendBaseData();
  uint8_t movingStateUpdate();
  void debugFunction();



  void cmd_callback(const geometry_msgs::Twist::ConstPtr& input) {
    cmd_vel_ = *input;
  }

  
};


#endif

