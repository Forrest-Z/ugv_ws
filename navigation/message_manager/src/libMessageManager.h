#ifndef LIB_MESSAGEMANAGER_H
#define LIB_MESSAGEMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using std::vector;
using std::cout;
using std::endl;
using std::isnan;
using std::string;
using std::to_string;

const double PI = 3.14159265359;

class MessageManager
{
public:
  MessageManager();
  ~MessageManager();
  void Initialization();
  void Execute();

private:
  const int ROS_RATE_HZ = 10;
  const int robot_number = 2;

  /** ROS Components **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  ros::Subscriber cmd_sub;
  ros::Subscriber reset_sub;
  ros::Subscriber station_sub;
  ros::Subscriber control_sub;
  ros::Subscriber action_state_sub;

  ros::Publisher command_pub;
  ros::Publisher center_pub;
  ros::Publisher heart_pub;

  ros::Publisher goal_pub;
  ros::Publisher action_pub;

  tf::TransformListener listener_map_to_base;

  /** Parameters **/
  string robot_id_;
  int id_number_;

  /** Flags **/
  /** Variables **/
  int robot_speed_;
  string robot_seq_;
  string robot_act_;

  string last_input_raw_;
  string last_input_sequence_;
  string last_input_control_;
  string last_input_control_sequence_;
  
  ros::Time run_timer_;
  /** Functions **/
  void ControlCallback(const std_msgs::String::ConstPtr& Input);
  void StationCallback(const std_msgs::String::ConstPtr& Input);
  void RecallCallback(const std_msgs::String::ConstPtr& Input);
  void ProcessMission(string Type,string Seq,string Command_1,string Command_2);
  void UpdateRealtimeInfo(ros::Time& Timer,double Freq);
  void UpdateTaskInfo(string Input = "01");

  void ActionStateCallback(const std_msgs::Int32::ConstPtr& Input) {
    // cout << robot_id_ << " ActionStateCallback" << endl;
    robot_act_ = FillInt2String(Input->data,2,1);
    UpdateTaskInfo("02");
  }

  void CommandCallback(const geometry_msgs::Twist::ConstPtr& Input) {
    robot_speed_ = int(Input->linear.x * 10);
    // if(robot_id_ == "robot_1") cout << robot_id_ << " Speed Update " << robot_speed_ << endl;
  }

  void ResetCallback(const std_msgs::Int32::ConstPtr& Input) {
    if(Input->data == 0 || Input->data == 1) Initialization();
  }

  string FillInt2String(int Input,int Digit,int Type) {
    string output;
    string pos0neg;
    if(Type == 0) {
      (Input < 0) ? pos0neg = "0" : pos0neg = "1";
      string input_val = to_string(abs(Input));
      output = pos0neg;
      for (int i = 0; i < Digit - input_val.size() -1; ++i) {
        output += "0";
      }
      output += input_val;
    }
    if(Type == 1){
      string input_val = to_string(abs(Input));
      for (int i = 0; i < Digit - input_val.size(); ++i) {
        output += "0";
      }
      output += input_val;      
    }
    return output;
  }


  int FindMissionType(string Input) {
    if(Input == "00" || Input == "01" || Input == "02" || Input == "08" ) return 1;
    if(Input == "06" || Input == "07") return 2;
    if(Input == "03" || Input == "04" || Input == "05") return 3;
    return -1;
  }

};

#endif
