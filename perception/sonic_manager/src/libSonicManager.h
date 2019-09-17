#ifndef LIB_SONICMANAGER_H
#define LIB_SONICMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>

#include <deque>
#include <queue> 


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
#include <geometry_msgs/Twist.h>

#include <kobuki_msgs/BumperEvent.h>


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
  ros::Publisher vel_pub;

  /** Subscribers **/
  ros::Subscriber bumper_sub;

  /** Parameters **/
  string port_name_;
  int baud_rate_;

  int emergency_range_;

  serial_port_ptr port_;
  boost::system::error_code error_code_;
  boost::asio::io_service io_service_;

  /** Flags **/

  /** Variables **/
  int sonic_state_;
  int bumper_state_;

  ros::Time clock_1;
  
  /** Functions **/
  void readBaseData(vector<int>& Output);
  void sendState();


  int makeDecision(vector<int> range);
  int makeMajorDecision(vector<int> range,int state);

  bool findSafeDirection(vector<int> Input,vector<int>& Output);
  void moveMyself(int Input);

  void calibrationSonic(vector<int> Input,vector<int>& Output);

  void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& Input) {
    if(Input->state == 1) bumper_state_ = -1;
    else bumper_state_ = 0;
  }

  vector<int> minusVectors(vector<int> vec_1,vector<int> vec_2) {
    vector<int> output;
    if(vec_1.size() != vec_2.size()){
      cout << "Unmatched Vector size" <<endl;
    } else {
      for (int i = 0; i < vec_1.size(); ++i) {
        output.push_back(vec_1[i] - vec_2[i]);
      }      
    }
    return output;
  }

  vector<int> plusVectors(vector<int> vec_1,vector<int> vec_2) {
    vector<int> output;
    if(vec_1.size() != vec_2.size()){
      cout << "Unmatched Vector size" <<endl;
    } else {
      for (int i = 0; i < vec_1.size(); ++i) {
        output.push_back(vec_1[i] + vec_2[i]);
      }      
    }
    return output;
  }

  vector<int> dividVectors(vector<int> vec,int factor) {
    vector<int> output;
    for (int i = 0; i < vec.size(); ++i) {
      output.push_back(vec[i]/factor);
    }      
    return output;
  }

  
};


#endif

