#include "libMqttNode.h"

MqttROS::MqttROS(string id,string host,vector<string> topic,int port,string community):
myMqtt(("robot_"+id).c_str(),host.c_str(),port,20,id,community)
{
  return_sub = n.subscribe("from_robot_task",10, &MqttROS::ReturnCallback,this);
  heartbeat_sub = n.subscribe("from_robot_status",10, &MqttROS::HeartbeatCallback,this);

  info_str_sub = n.subscribe("from_robot_info",10, &MqttROS::InfoCallback,this);
  scan_str_sub = n.subscribe("from_robot_lidar",10, &MqttROS::ScanCallback,this);
  plan_str_sub = n.subscribe("from_robot_path",10, &MqttROS::PlanCallback,this);
  exp_str_sub = n.subscribe("from_robot_exception",10, &MqttROS::ExpCallback,this);


  command_pub = n.advertise<std_msgs::String>("to_robot_task",5);
  control_pub = n.advertise<std_msgs::String>("to_robot_control",5);
  // mqtt_command_str_ = "CMD0$0$";

  robot_id_ = id;
  community_id_ = community;
  cout << "MqttNode Initialized " << robot_id_ << endl;
  mqtt_command_topic_   = topic[0];
  mqtt_control_topic_   = topic[1];

  mqtt_heartbeat_topic_ = topic[2];
  mqtt_return_topic_    = topic[3];
  mqtt_info_topic_      = topic[4];
  mqtt_path_topic_      = topic[5];
  mqtt_scan_topic_      = topic[6];
  mqtt_exp_topic_       = topic[7];
}

MqttROS::~MqttROS(){
}

void MqttROS::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void MqttROS::Mission() {
//	string mqtt_command_topic = "mqtt_command";
//	publish(NULL, mqtt_command_topic.c_str(), strlen(mqtt_command_str_.c_str()), mqtt_command_str_.c_str(), 1, false);
	
}
