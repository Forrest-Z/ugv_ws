#ifndef LIB_GPBASE_H
#define LIB_GPBASE_H

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

const double WHEEL_RADIUS_M = 0.165/2;
const double WHELL_BASE_M = 0.59;

const double speed_scale = 102/19;

const int HISTORY_SIZE  = 20;


typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


class GPBase
{
public:
  GPBase();
  ~GPBase();

  void Manager();
  void Mission();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/
  ros::Publisher odom_pub;
  ros::Publisher battery_pub;


  /** Subscribers **/
  ros::Subscriber cmd_sub;

  /** Parameters **/
  string port_name_;
  int baud_rate_;

  serial_port_ptr port_;
  boost::system::error_code error_code_;
  boost::asio::io_service io_service_;

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time last_time_;
  ros::Time current_time_;

  /** Flags **/
  bool isGetFirstData_;
  bool isCmdUpdate_;


  /** Variables **/
  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::Twist base_twist_;
  nav_msgs::Odometry base_odom_;

  list<float> battery_history_;

  /** Functions **/
  void sendBaseData();
  void readBaseData();
  bool checkDataHead(vector<unsigned char> input);
  void publishOdom();
  void debugFunction();

  vector<unsigned char> num2hex(int Input);
  int hex2num(vector<unsigned char> Input);

  void read_some_handler(const boost::system::error_code& error,std::size_t bytes_transferred) {
    cout << "handler" << endl;
    // sendBaseData();
  }

  double rpm2ms(int Input){
    return (Input * 2 * PI * WHEEL_RADIUS_M)/(60 * speed_scale);
  }

  int ms2rpm(double Input){
    return (Input * 60 * speed_scale)/(2 * PI * WHEEL_RADIUS_M);
  }

  int computeDigit(int Input){
    if (Input == 0) return 1;
    return 1 + int(log10(fabs(Input)));
  }

  float checkBatteryAverage() {
    float output;
    for (auto const& val : battery_history_) {
      output += val;
    }
    output /= battery_history_.size();
    return output;
  }

  void addBatteryHistory(float Input) {
    if(battery_history_.size() > HISTORY_SIZE) battery_history_.pop_front();
    battery_history_.push_back(Input);
  }


  string fillZero(std::string input,int Dig) {
    std::string output;
    for (int i = 0; i < Dig - input.size(); ++i) {
      output += "0";
    }
    output += input;
    return output;
  }

  void async_write_handler(const boost::system::error_code& error,std::size_t bytes_transferred) {
    cout << "async_write_handler" << endl;
    // sendBaseData();
  }

  void cmd_callback(const geometry_msgs::Twist::ConstPtr& input) {
    cmd_vel_ = *input;
    isCmdUpdate_ = true;
    // if(cmd_vel_.linear.z == 1){
    //   base_odom_.pose.pose.position.x = 0;
    //   base_odom_.pose.pose.position.y = 0;
    //   base_odom_.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(0);
    // }
  }

  
};


#endif

