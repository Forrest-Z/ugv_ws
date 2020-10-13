#ifndef LIB_GPLOADER_H
#define LIB_GPLOADER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>
#include <cfloat>

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
#include <geometry_msgs/Point32.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <bitset> 


using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::to_string;
using std::list;

const int ROS_RATE_HZ   = 5;
const double PI = 3.14159265359;

const int MAX_16BIT = 65535;


typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


class GPLoader
{
public:
  GPLoader();
  ~GPLoader();

  void Manager();
  void Mission();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/

  /** Subscribers **/
  ros::Subscriber cmd_sub;
  ros::Subscriber level_sub;

  /** Parameters **/
  string port_name_;
  int baud_rate_;

  serial_port_ptr port_;
  boost::system::error_code error_code_;
  boost::asio::io_service io_service_;

  ros::Time last_time_;
  ros::Time current_time_;

  /** Flags **/

  /** Variables **/
  int loader_gear_;
  int loader_level_;

  /** Functions **/
  void sendBaseData();
  void readBaseData();
  bool checkDataHead(vector<unsigned char> Input);
  void debugFunction();

  vector<unsigned char> num2hex(int Input);
  int hex2num(vector<unsigned char> Input);

  void read_some_handler(const boost::system::error_code& error,std::size_t bytes_transferred) {
    cout << "handler" << endl;
    // sendBaseData();
  }

  int computeDigit(int Input){
    if (Input == 0) return 1;
    return 1 + int(log10(fabs(Input)));
  }

  string fillZero(std::string Input,int Dig) {
    std::string output;
    for (int i = 0; i < Dig - Input.size(); ++i) {
      output += "0";
    }
    output += Input;
    return output;
  }

  void async_write_handler(const boost::system::error_code& error,std::size_t bytes_transferred) {
    cout << "async_write_handler" << endl;
    // sendBaseData();
  }


  void cmd_callback(const geometry_msgs::Twist::ConstPtr& Input) {
    // cout << "command received" << endl;
    int max_gear = 9;
    int max_level = 15;
    int speed_unit = 20;
    int level_unit = 10;

    loader_level_ += static_cast<int>(Input->linear.x);
    if(loader_level_ > max_level) loader_level_ = max_level;
    if(loader_level_ < 0) loader_level_ = 0;

    loader_gear_ += static_cast<int>(Input->linear.y);
    if(fabs(loader_gear_) > max_gear) loader_gear_ = loader_gear_/fabs(loader_gear_) * max_gear;

    if(Input->linear.z == -1) loader_gear_ = 0;

loader_gear_ *= speed_unit;
loader_level_ *= level_unit;
	

    // cout << "Loader gear  : " << loader_gear_ << endl;
    // cout << "Loader level : " << loader_level_ << endl;
  }

  void level_callback(const geometry_msgs::Twist::ConstPtr& Input) {
    // cout << "command received" << endl;
    int max_speed = 180;
    int max_angle = 149;
    int min_angle = 50;
    loader_level_ = static_cast<int>(Input->linear.x);
    if(loader_level_ > max_angle) loader_level_ = max_angle;
    if(loader_level_ < min_angle) loader_level_ = 0;


    loader_gear_ = static_cast<int>(Input->linear.y);
    if(fabs(loader_gear_) > max_speed) loader_gear_ = loader_gear_/fabs(loader_gear_) * max_speed;

    // cout << "Loader gear  : " << loader_gear_ << endl;
    // cout << "Loader level : " << loader_level_ << endl;
  }

  
};


#endif

