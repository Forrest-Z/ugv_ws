#ifndef LIB_GPBASE_H
#define LIB_GPBASE_H

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


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <bitset> 


using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::to_string;

const int ROS_RATE_HZ   = 20;
const double PI = 3.14159265359;

const double WHEEL_RADIUS_M = 0.165;
const double WHELL_BASE_M = 0.59;

const double speed_scale = 8;


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
  /** Functions **/
  void sendBaseData();
  void readBaseData();
  bool checkDataHead(vector<char>& input);
  void publishOdom();
  void debugFunction();

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

  vector<unsigned char> num2hex(int Input){
    vector<unsigned char> Output;
    if(Input >= 0) {
      std::stringstream ss;
      ss << std::hex << Input;
      string output_str = ss.str();
      for (int i = 0; i < output_str.size()-4; ++i) {
        output_str = "0" + output_str;
      }

      string part_1 = output_str.substr(0,2);
      string part_2 = output_str.substr(2,2);


      int part_1_int = stoi(part_1,nullptr,16);
      int part_2_int = stoi(part_2,nullptr,16);

      unsigned char part_1_char = part_1_int;
      unsigned char part_2_char = part_2_int; 


      Output.push_back(part_2_char);
      Output.push_back(part_1_char);

      // cout << "Input    :" << Input << endl;
      // cout << "part_1   :" << part_1 << endl;
      // cout << "part_2   :" << part_2 << endl; 
      // cout << "char_1   :" << int(part_1_char) << endl;
      // cout << "char_2   :" << int(part_2_char) << endl;
      // cout << "output_1 :" << int(Output[0]) << endl;
      // cout << "output_2 :" << int(Output[1]) << endl; 
    } else {
      std::stringstream ss;
      ss << std::hex << abs(Input);
      string output_str = ss.str();
      for (int i = 0; i < output_str.size()-4; ++i) {
        output_str = "0" + output_str;
      }

      int before_int = stoi(output_str,nullptr,16);
      std::bitset<16> input_bin(before_int);
      input_bin = ~input_bin;
      int after_int = stoi(input_bin.to_string(),nullptr,2);
      after_int++;

      std::stringstream ss_after;
      ss_after << std::hex << after_int;
      string output_str_after = ss_after.str();

      for (int i = 0; i < output_str_after.size()-4; ++i) {
        output_str_after = "0" + output_str_after;
      }

      string part_1 = output_str_after.substr(0,2);
      string part_2 = output_str_after.substr(2,2);


      int part_1_int = stoi(part_1,nullptr,16);
      int part_2_int = stoi(part_2,nullptr,16);

      unsigned char part_1_char = part_1_int;
      unsigned char part_2_char = part_2_int; 

      Output.push_back(part_2_char);
      Output.push_back(part_1_char);
      // cout << "Input    :" << Input << endl;
      // cout << "String   :" << after_string << endl;
      // cout << "part_1   :" << part_1 << endl;
      // cout << "part_2   :" << part_2 << endl; 
      // cout << "char_1   :" << int(part_1_char) << endl;
      // cout << "char_2   :" << int(part_2_char) << endl;
      // cout << "output_1 :" << int(Output[0]) << endl;
      // cout << "output_2 :" << int(Output[1]) << endl; 
    }





    return Output;

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

