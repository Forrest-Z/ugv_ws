#ifndef LIB_LOGMANAGER_H
#define LIB_LOGMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>

#include <std_msgs/String.h>

using std::vector;
using std::list;
using std::cout;
using std::endl;
using std::isnan;
using std::string;
using std::to_string;

const double PI = 3.14159265359;


struct LogInfo {
  string date;
  string seq;
  string from;
  string to;
  string command;
  string state;
};



class LogManager
{
public:
  LogManager();
  ~LogManager();
  void Initialization();
  void Execute();

private:
  const int ROS_RATE_HZ = 100;

  /** ROS Components **/
  ros::NodeHandle n;

  ros::Subscriber center_sub;
  ros::Subscriber station_sub;

  /** Parameters **/

  /** Variables **/
  int history_size_;
  list<string> command_history_;

  /** Functions **/
  void ReceivedCallback(const std_msgs::String::ConstPtr& Input);
  void UploadCallback(const std_msgs::String::ConstPtr& Input);
  void PrintLogList(LogInfo Input);

  bool ReadytoPrint(ros::Time Input,double Threshold) {
    if((ros::Time::now() - Input).toSec() > Threshold) return true;
    return false;
  }

  void AddHistroy(string Input) {
    if(command_history_.size() > history_size_) command_history_.pop_front();
    command_history_.push_back(Input);
  }

  bool CheckDuplicateHistory(string Input){
    for (auto const& history_data : command_history_) {
      if(history_data == Input) return true;
    }
    return false;
  }

  string SaveCurrentDate() {
    string Output;
    time_t now = time(0);
    tm *ltm = localtime(&now);
    Output = FillStringWithZero(to_string(ltm->tm_hour),2) + ":" + FillStringWithZero(to_string(ltm->tm_min),2) + ":" + FillStringWithZero(to_string(ltm->tm_sec),2);
    return Output;
  }

  string ConvertMissionDetial2English(string Command,string Type) {
    string Output;
    int Action = FindMissionType(Type);
    if(Action < 0) {
      Output = "Action Unknown";
    } 
    if(Action == 1) {
      string area_id = Command.substr(0,2);
      string building_number = Command.substr(2,2);
      string gate_number = Command.substr(6,2);
      Output = area_id + "-" + building_number + "-" + gate_number;
    }
    if(Action == 0) Output = "Waiting";
    if(Action == 3) Output = "Action";
    return Output;

  }

  int FindMissionType(string Input) {
    if(Input == "00" || Input == "01" || Input == "02" || Input == "08" ) return 1;
    if(Input == "06" || Input == "07") return 0;
    if(Input == "03" || Input == "04" || Input == "05") return 3;
    return -1;
  }


  string ConvertMissionType2English(string Input) {
    string Output;
    if(Input == "00")      Output = "Delivery ";
    else if(Input == "01") Output = "Pick Item";
    else if(Input == "02") Output = "Pick Box ";
    else if(Input == "03") Output = "Drop Box ";
    else if(Input == "04") Output = "Load Item";
    else if(Input == "05") Output = "Load Box ";
    else if(Input == "06") Output = "Standby  ";
    else if(Input == "07") Output = "Freeze   ";
    else if(Input == "08") Output = "Go Home  ";
    else Output = "Unkonwn Mission";
    return Output;
  }

  string ConvertMissionStatus2English(string Input) {
    string Output;
    if(Input == "00")      Output = "Error";
    else if(Input == "01") Output = "Start";
    else if(Input == "02") Output = "Done ";
    else Output = "Unkonwn Status";
    return Output;
  }


  string FillStringWithZero(string Input,int Digit) {
    string Output;
    int digit_short = Digit - Input.size();
    for (int i = 0; i < digit_short; ++i) {
      Output += "0";
    }
    Output += Input;
    return Output;
  }



  string FillInt2String(int Input,int Digit,int Type) {
    string Output;
    string pos0neg;

    if(Type == 0) {
      (Input < 0) ? pos0neg = "0" : pos0neg = "1";
      string input_val = to_string(abs(Input));
      Output = pos0neg;
      for (int i = 0; i < Digit - input_val.size() -1; ++i) {
        Output += "0";
      }
      Output += input_val;
    }

    if(Type == 1){
      string input_val = to_string(abs(Input));
      for (int i = 0; i < Digit - input_val.size(); ++i) {
        Output += "0";
      }
      Output += input_val;      
    }

    return Output;
  }






};

#endif
