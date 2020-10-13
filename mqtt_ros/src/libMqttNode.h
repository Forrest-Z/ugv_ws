#ifndef LIB_MQTT_H
#define LIB_MQTT_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <csignal>

#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <mosquittopp.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::to_string;
using std::list;

const int ROS_RATE_HZ   = 20;
const double PI = 3.14159265359;


class myMqtt : public mosqpp::mosquittopp
{
public:
  myMqtt(){
  };
  myMqtt(const char* id, const char* host, int port,int keepalive,string robot_id,string building_id):
    mosquittopp(id)
  {
    max_payload_ = 300;
    mosqpp::lib_init();
    connect_async(host, port, keepalive);
    loop_start();
    ROS_INFO_ONCE("Mqtt Connection Init");
    isMqttReady_ = false;
    mqtt_download_topic_ = "robot/" + building_id + "/" + robot_id + "/download/#";
  }

  ~myMqtt() {
    close_connect();
  }

  void on_connect(int rc){
    if ( rc == 0 ) {
      ROS_INFO_ONCE("myMqtt - connected with server");
      this->startSubscribe();
      isMqttReady_ = true;
    } else {
      ROS_WARN_STREAM_ONCE("myMqtt - Impossible to connect with server(" << rc << ")");
      isMqttReady_ = false;
    }
  }

  void on_disconnect(int rc){
    ROS_WARN_STREAM("myMqtt - disconnection(" << rc << ")");
    isMqttReady_ = false;
    this->reconnect();
  }

  void close_connect() {
    cout << "Mqtt Connection Closed" << endl;
    loop_stop(true);
    mosqpp::lib_cleanup();
  }

  void startSubscribe() {
    // cout << "Mqtt Subscribe Topic : " << mqtt_download_topic_ << endl;
    this->subscribe(NULL, mqtt_download_topic_.c_str());
  }

  double max_payload_;
  string mqtt_download_topic_;

  bool isMqttReady_;

private: 


};


class MqttROS : public myMqtt
{
public:
  MqttROS(string id,string host,vector<string> topic,int port,string community);
  ~MqttROS();

  void Manager();
  void Mission();


private:
  const int BUTTON_LB             = 4;
  const int BUTTON_RB             = 5;

  const int BUTTON_A              = 0;
  const int BUTTON_B              = 1;
  const int BUTTON_X              = 2;
  const int BUTTON_Y              = 3;

  const int BUTTON_BACK           = 6;
  const int BUTTON_START          = 7;

  const int AXIS_LEFT_LEFT_RIGHT  = 0;
  const int AXIS_LEFT_UP_DOWN     = 1;
  const int AXIS_RIGHT_LEFT_RIGHT = 3;
  const int AXIS_RIGHT_UP_DOWN    = 4;

  const int AXIS_CROSS_LEFT_RIGHT = 6;
  const int AXIS_CROSS_UP_DOWN    = 7;

  /** Node Handles **/
  ros::NodeHandle n;

  /** Publishers **/
  ros::Publisher command_pub;
  ros::Publisher control_pub;

  /** Subscribers **/
  ros::Subscriber return_sub;
  ros::Subscriber heartbeat_sub;

  ros::Subscriber scan_str_sub;
  ros::Subscriber info_str_sub;
  ros::Subscriber plan_str_sub;
  ros::Subscriber exp_str_sub;


  /** Variables **/
  string mqtt_command_topic_;
  string mqtt_control_topic_;

  string mqtt_heartbeat_topic_;
  string mqtt_return_topic_;
  string mqtt_info_topic_;
  string mqtt_scan_topic_;
  string mqtt_path_topic_;
  string mqtt_exp_topic_;
  
  bool isManualControl_;
  string mqtt_command_str_;

  int heartbeat_id_pos_;
  int command_id_pos_;
  int return_id_pos_;
  string robot_id_;
  string community_id_;

  /** Functions **/
  void on_message(const struct mosquitto_message *message) {
    int payload_size = max_payload_ + 1;
    char buf[payload_size];
    string mqtt_sub_header = "robot/" + community_id_ + "/" + robot_id_ + "/download/";

    string mqtt_task_topic = mqtt_sub_header + mqtt_command_topic_;
    string mqtt_control_topic = mqtt_sub_header + mqtt_control_topic_;

    // cout << "mqtt_task_topic    : " << mqtt_task_topic << endl;
    // cout << "mqtt_control_topic : " << mqtt_control_topic << endl;
    cout << "robot_" << robot_id_ << " Received topic : " << message->topic << endl;

    if(!strcmp(message->topic, mqtt_task_topic.c_str()))
    {
      memset(buf, 0, payload_size * sizeof(char));
      memcpy(buf, message->payload, max_payload_ * sizeof(char));

      string message_str = buf;
      cout << "robot_" << robot_id_ << " Task Received: " << buf << endl;
      std_msgs::String temp_str;
      temp_str.data = buf;
      command_pub.publish(temp_str);
    }

    if(!strcmp(message->topic, mqtt_control_topic.c_str()))
    {
      memset(buf, 0, payload_size * sizeof(char));
      memcpy(buf, message->payload, max_payload_ * sizeof(char));
      // ROS_INFO("Timer");
      cout << robot_id_ << " Control Received: "<< buf << endl;
      std_msgs::String temp_str;
      temp_str.data = buf;
      control_pub.publish(temp_str);
    }

    // Test
    // memset(buf, 0, payload_size * sizeof(char));
    // memcpy(buf, message->payload, max_payload_ * sizeof(char));
    // ROS_INFO_STREAM("Test received from " << message->topic << " : "<< buf);
  }


  void ReturnCallback(const std_msgs::String::ConstPtr& Input) {
    if(!isMqttReady_) return;
    string message_str = Input->data;
    string mqtt_topic = "robot/" + community_id_ + "/" + robot_id_ + "/upload/" + mqtt_return_topic_;
    publish(NULL, mqtt_topic.c_str(), strlen(message_str.c_str()), message_str.c_str(), 2, false);
    cout << "robot_" << robot_id_ << " Return Topic : " << mqtt_topic << " - data : " << message_str <<endl;
  }

  void HeartbeatCallback(const std_msgs::String::ConstPtr& Input) {
    if(!isMqttReady_) return;
    string message_str = Input->data;
    string mqtt_topic = "robot/" + community_id_ + "/" + robot_id_ + "/upload/" + mqtt_heartbeat_topic_;
    publish(NULL, mqtt_topic.c_str(), strlen(message_str.c_str()), message_str.c_str(), 0,false);
    // cout << "robot_" << robot_id_  << "Heartbeat Topic : " << mqtt_topic << " - data : " << message_str <<endl;
  }

  void ScanCallback(const std_msgs::String::ConstPtr& Input) {
    if(!isMqttReady_) return;
    string message_str = Input->data;
    string mqtt_topic = "robot/" + community_id_ + "/" + robot_id_ + "/upload/" + mqtt_scan_topic_;
    publish(NULL, mqtt_topic.c_str(), strlen(message_str.c_str()), message_str.c_str(), 0,false);
    // cout << "Scan Topic : " << mqtt_topic << " - data : " << message_str <<endl;
  }

  void PlanCallback(const std_msgs::String::ConstPtr& Input) {
    if(!isMqttReady_) return;
    string message_str = Input->data;
    string mqtt_topic = "robot/" + community_id_ + "/" + robot_id_ + "/upload/" + mqtt_path_topic_;
    publish(NULL, mqtt_topic.c_str(), strlen(message_str.c_str()), message_str.c_str(), 2,false);
    cout << robot_id_  << "Plan Topic : " << mqtt_topic << " - data : " << message_str <<endl;
  }

  void InfoCallback(const std_msgs::String::ConstPtr& Input) {
    if(!isMqttReady_) return;
    string message_str = Input->data;
    string mqtt_topic = "robot/" + community_id_ + "/" + robot_id_ + "/upload/" + mqtt_info_topic_;
    publish(NULL, mqtt_topic.c_str(), strlen(message_str.c_str()), message_str.c_str(), 0,false);
    // cout << "Info Topic : " << mqtt_topic << " - data : " << message_str <<endl;
  }

  void ExpCallback(const std_msgs::String::ConstPtr& Input) {
    if(!isMqttReady_) return;
    string message_str = Input->data;
    string mqtt_topic = "robot/" + community_id_ + "/" + robot_id_ + "/upload/" + mqtt_exp_topic_;
    publish(NULL, mqtt_topic.c_str(), strlen(message_str.c_str()), message_str.c_str(), 2,false);
    // cout << "Exception Topic : " << mqtt_topic << " - data : " << message_str <<endl;
  }


};


#endif

