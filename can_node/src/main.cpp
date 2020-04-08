#include "can_interface.hpp"
#include <ros/ros.h>
#include <iostream>
#include "rover_msgs/can.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 
#include <stdio.h>

#include <sstream>
#include <string>

const int ROS_RATE_HZ = 20;

using namespace std;
rover_msgs::can can_msg;
void * Callback_Read(struct canfd_frame frame,void * operatePtr)
{   
  // ROS_INFO("Callback_Read active");
  can_msg.header.stamp = ros::Time::now();
  can_msg.header.frame_id = "RPM";

  if(frame.can_id==0x301) {
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  if(frame.can_id==0x702) {
    // int rpm = int(frame.data[3]);
    cout << "ID 702" << endl;
  }




/**
  if(frame.can_id==0x301) {
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  if(frame.can_id==0x520) {
    int rpm = int(frame.data[2])+int(frame.data[3])*256;
    if(rpm>65535/2)
        can_msg.rpm.data = 65535-rpm;
    else
        can_msg.rpm.data=-rpm;

    ROS_INFO("Rpm %f",can_msg.rpm.data);
  }
  if(frame.can_id==0x401){
    can_msg.angle.data = int(frame.data[3])*256-1024+int(frame.data[4]);
    ((ros::Publisher*)operatePtr)->publish(can_msg);
    ROS_INFO("Steering %f",can_msg.angle.data);
  }
**/



}


int convertInt2Bin(int input){
  int output;
  int input_temp = input;
  for(int i = 0; input_temp > 0; i++)
  {
    output += input_temp % 2 * pow(10,i);
    input_temp = input_temp / 2;
  }
  return output;
}


int convertBin2Int(int input) 
{ 
    int input_temp = input; 
    int output;
    int base = 1;  
    while (input_temp) { 
      int last_digit = input_temp % 10; 
      input_temp = input_temp / 10; 
      output += last_digit * base; 
      base = base * 2; 
    } 
    return output; 
}


char *generateData(int speed,int steering) {
  const char *can_id="301#";
  char *output;
  char *data_stream;
  int temp_int;

  int input[7];

  char buf_mode[4];
  (speed < 0) ? temp_int = 2 : temp_int = 1;
  input[0] = 10 * temp_int;
  sprintf(buf_mode,"%01x",temp_int);

  char buf_gear[4];
  temp_int = 1;
  input[0] += temp_int;
  sprintf(buf_gear,"%01x",temp_int);
  
  char buf_angle[8];
  temp_int = steering;
  input[1] = temp_int;
  sprintf(buf_angle,"%02x",temp_int);
  
  char buf_angular[8];
  temp_int = 200;
  input[2] = temp_int;
  sprintf(buf_angular,"%02x",temp_int);
  
  char buf_speed[8];
  temp_int = speed;
  input[3] = temp_int;
  sprintf(buf_speed,"%02x",temp_int);
  
  char buf_empty[8];
  temp_int = 0;
  input[4] = temp_int;
  sprintf(buf_empty,"%02x",temp_int);

  char buf_light[4];
  temp_int = 8;
  input[5] = temp_int;
  sprintf(buf_light,"%02x",temp_int);

  char buf_bumper[4];
  temp_int = 0;
  input[6] = temp_int;
  sprintf(buf_bumper,"%02x",temp_int);

  char buf_cab[8];
  temp_int = 0;
  input[7] = temp_int;
  sprintf(buf_cab,"%02x",temp_int);


  char buf_check[8];
  for (int i = 0; i < 7; i++) {
    temp_int ^= input[i];
  }
  sprintf(buf_check,"%02x",temp_int);


  asprintf(&data_stream, "%s%s%s%s%s%s%s%s%s%s",buf_mode,buf_gear,buf_angle,buf_angular,buf_speed,buf_empty,buf_light,buf_bumper,buf_cab,buf_check);
  asprintf(&output, "%s%s",can_id,data_stream,buf_speed);

  cout << "buf_mode:" << buf_mode << endl;  
  cout << "buf_gear:" << buf_gear << endl;
  cout << "buf_angle:" << buf_angle << endl;
  cout << "buf_angular:" << buf_angular << endl;
  cout << "buf_speed:" << buf_speed << endl;
  cout << "buf_empty:" << buf_empty << endl;
  cout << "buf_empty:" << buf_empty << endl;
  cout << "buf_light:" << buf_light << endl;
  cout << "buf_bumper:" << buf_bumper << endl;
  cout << "buf_cab:" << buf_cab << endl;
  cout << "buf_check:" << buf_check << endl<<endl;

  return output;
}


void CmdCallback(const geometry_msgs::Twist::ConstPtr& Input) {
  double speed = Input->linear.x;
  double steering = Input->angular.z;
  double max_steering = 1.5;

  int speed_input = speed / 0.05;
  int steering_input = 35;
  if(fabs(steering) > 0.01) {
    steering_input = (steering/1.5) * 35 + 35;
  }
  
  cout << "speed_input:" << speed_input << endl;
  cout << "steering_input:" << steering_input << endl;

  char *can_data = generateData(speed_input,steering_input);

  cout << "Output:" << can_data << endl;
  // writeData(can_data);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "can_send_node");
  ros::NodeHandle nh;


  ros::Publisher pub_of_can = nh.advertise<rover_msgs::can>("can_pub", 1);
  ros::Subscriber cmd_sub = nh.subscribe("/husky_velocity_controller/cmd_vel",1, &CmdCallback);
  // readData((void*)&pub_of_can);
  ros::spin();
  // while (ros::ok()) {
  //   ros::spinOnce();
  // }

}