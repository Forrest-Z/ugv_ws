#include "ros/ros.h"
#include "libMqttNode.h"


int main(int argc, char ** argv) {

    ros::init(argc, argv, "mqtt_ros_node");
    ros::NodeHandle n;
  	ros::NodeHandle pn("~");

  	string robot_id,host_address,mqtt2bot_namespace,bot2mqtt_namespace;
  	int port_num;

    vector<string> mqtt_topics(6);

  	n.param<string>("robot_id", robot_id, "robot_3");
  	pn.param<string>("host_address",host_address,"121.40.153.189");
  	pn.param<string>("mqtt_command_topic",mqtt_topics[0],"task");
    pn.param<string>("mqtt_control_topic",mqtt_topics[1],"control");

    pn.param<string>("mqtt_heartbeat_topic",mqtt_topics[2],"status");
    pn.param<string>("mqtt_return_topic",mqtt_topics[3],"task");

    pn.param<string>("mqtt_return_topic",mqtt_topics[4],"navi");
    pn.param<string>("mqtt_return_topic",mqtt_topics[5],"lidar");
  	pn.param<int>("port_num", port_num, 1883);


    if(robot_id.size() == 7){
      robot_id = robot_id.substr(0,6) + "0" + robot_id.substr(6,1);
    }

    MqttROS MyControl(robot_id,host_address,mqtt_topics,port_num);

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