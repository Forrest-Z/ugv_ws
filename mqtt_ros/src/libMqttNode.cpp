#include "libMqttNode.h"

MqttROS::MqttROS(string id,string host,vector<string> topic,int port):
myMqtt(id.c_str(),host.c_str(),port,20)
{
  return_sub = n.subscribe("/robot_backend_task",10, &MqttROS::ReturnCallback,this);
  heartbeat_sub = n.subscribe("/robot_backend_info",10, &MqttROS::HeartbeatCallback,this);

  scan_str_sub = n.subscribe("scan_str",10, &MqttROS::ScanCallback,this);
  // joy_sub = n.subscribe("/joy",1, &MqttROS::JoyCallback,this);


  command_pub = n.advertise<std_msgs::String>("/to_robot_task",1);

  control_pub = n.advertise<std_msgs::String>("to_robot_control",1);
  // mqtt_command_str_ = "CMD0$0$";

  std::size_t pos = id.find("_");
  node_id_ = id.substr(pos+1);

  cout << "node_id_ : " << node_id_ << endl;
  mqtt_command_topic_   = topic[0];
  mqtt_control_topic_   = topic[1];
  mqtt_heartbeat_topic_ = topic[2];
  mqtt_return_topic_    = topic[3];
  mqtt_navi_topic_      = topic[4];
  mqtt_scan_topic_      = topic[5];

  command_id_pos_ = 10;
  return_id_pos_ = 12;
  heartbeat_id_pos_ = 12;
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
