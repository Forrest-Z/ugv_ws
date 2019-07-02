#ifndef LIB_JCBASE_H
#define LIB_JCBASE_H

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

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::to_string;

const int ROS_RATE_HZ   = 20;
const double PI = 3.14159265359;

const double WHEEL_RADIUS_M = 0.165;
const double WHELL_BASE_M = 0.59;

const double speed_scale = 4.2;
const double rotation_scale = 0.45;

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


class JCBase
{
public:
  JCBase();
  ~JCBase();

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

  vector<char> num2hex(int Input){

    int output_size = computeDigit(Input);
    if(Input < 0) output_size++;

    int write_ct = 0;

    vector<char> Output(output_size);

    if(Input == 0){
      Output[0] = 0x30;
      return Output;
    }

    if(Input < 0) {
      Output[0] = 0x2d;
      write_ct++;
    }
    int total_num = fabs(Input);
    for (int i = 0; i < computeDigit(Input); ++i){
      int num = total_num%10;
      Output[output_size - i - 1] = 0x30 + num;
      total_num /= 10;
      write_ct++;
    }

    return Output;
  }

  void cmd_callback(const geometry_msgs::Twist::ConstPtr& input) {
    cmd_vel_ = *input;
    isCmdUpdate_ = true;

    if(cmd_vel_.linear.z == 1){
      base_odom_.pose.pose.position.x = 0;
      base_odom_.pose.pose.position.y = 0;
      base_odom_.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(0);
    }
  }

  
};


#endif

