#include "ros/ros.h"
#include "libMqttNode.h"


int main(int argc, char ** argv) {

    ros::init(argc, argv, "mqtt_ros_node");
    ros::NodeHandle n;
  	ros::NodeHandle pn("~");

  	string community_id,robot_id,host_address,mqtt2bot_namespace,bot2mqtt_namespace;
  	int port_num;

    vector<string> mqtt_topics(8);

  	n.param<string>("robot_id", robot_id, "robot_12");
    n.param<string>("community_id", community_id, "1");

  	pn.param<string>("host_address",host_address,"121.40.153.189");
    
  	pn.param<string>("mqtt_command_topic",mqtt_topics[0],"dispatch/task");
    pn.param<string>("mqtt_control_topic",mqtt_topics[1],"control/basic");

    pn.param<string>("mqtt_heartbeat_topic",mqtt_topics[2],"dispatch/status");
    pn.param<string>("mqtt_return_topic",mqtt_topics[3],"dispatch/task");
    pn.param<string>("mqtt_info_topic",mqtt_topics[4],"navigation/info");
    pn.param<string>("mqtt_navi_topic",mqtt_topics[5],"navigation/path");
    pn.param<string>("mqtt_lidar_topic",mqtt_topics[6],"navigation/lidar");
    pn.param<string>("mqtt_exception_topic",mqtt_topics[7],"special/exception");

  	pn.param<int>("port_num", port_num, 1883);


    std::size_t pos = robot_id.find("_");
    robot_id = robot_id.substr(pos+1);
    robot_id = to_string(stoi(robot_id));
    
    MqttROS MyControl(robot_id,host_address,mqtt_topics,port_num,community_id);

    MyControl.Manager();

    return 0;
}

// robot/03/download/task
/*

For sub mqtt topic
mosquitto_sub -h 121.40.78.22 -v -t 'heartbeat_topic' 

mosquitto_pub -h 121.40.78.22 -t 'command_topic' -m 'msg1'

mosquitto_sub -h 192.168.100.55 -v -t 'robot/00/upload/status'

mosquitto_pub -h 192.168.100.55 -t 'command_topic' -m 'msg1'

*/