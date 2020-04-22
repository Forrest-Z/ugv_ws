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

  ros::Publisher center_pub;
  ros::Subscriber station_sub;

  ros::Publisher station_pub;
  ros::Publisher action_pub;
  ros::Publisher sequence_pub;
  ros::Publisher command_pub;

  ros::Subscriber cmd_sub;
  ros::Subscriber seq_sub;
  ros::Subscriber status_sub;
  ros::Subscriber action_sub;


  tf::TransformListener listener_map_to_base;

  /** Parameters **/
  string robot_id_;
  int id_number_;

  /** Flags **/
  /** Variables **/
  int robot_speed_;
  string robot_seq_;
  string robot_act_;
  int robot_status_;
  vector<bool> robot_update_;

  ros::Time run_timer_;
  /** Functions **/
  void StationCallback(const std_msgs::String::ConstPtr& Input);
  int FindMissionType(string Input);
  bool ProcessMission(string Command,string Type,string Seq);
  void UpdateRealtimeInfo();
  void UpdateTaskInfo();

  void CommandCallback(const geometry_msgs::Twist::ConstPtr& Input) {
    robot_speed_ = int(Input->linear.x * 100);
  }
  void SequenceCallback(const std_msgs::String::ConstPtr& Input) {
    // cout << "Sequence Update" << endl;
    robot_seq_ = Input->data;
    robot_update_[0] = true;
  }
  void ActionCallback(const std_msgs::String::ConstPtr& Input) {
    // cout << "Action Update" << endl;
    robot_act_ = Input->data;
    robot_update_[1] = true;
  }
  void StatusCallback(const std_msgs::Int32::ConstPtr& Input) {
    // cout << "Status Update" << endl;
    robot_status_ = Input->data;
    (Input->data == 2) ? robot_update_[3] = true:robot_update_[2] = true;
  }



  string ConvertMissionType2English(string Input) {
    string Output;
    if(Input == "00") Output = "Delivery";
    else if(Input == "01") Output = "Pickup Item";
    else if(Input == "02") Output = "Pickup Box";
    else if(Input == "03") Output = "Droping Box";
    else if(Input == "04") Output = "Loading Item";
    else if(Input == "05") Output = "Loading Box";
    else if(Input == "06") Output = "Standby";
    else if(Input == "07") Output = "Freeze";
    else if(Input == "08") Output = "Going Home";
    else Output = "Unkonwn Mission";
    return Output;
  }

  string ConvertMissionStatus2English(string Input) {
    string Output;
    if(Input == "00") Output = "Error";
    else if(Input == "01") Output = "Start";
    else if(Input == "02") Output = "Done";
    else Output = "Unkonwn Status";
    return Output;
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






};

#endif
