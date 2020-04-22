#include "libMessageManager.h"

MessageManager::MessageManager():pn("~") {
  n.param<string>("robot_id", robot_id_, "");
  cmd_sub = n.subscribe("husky_velocity_controller/cmd_vel",1, &MessageManager::CommandCallback,this);
  seq_sub = n.subscribe("robot_sequence",1, &MessageManager::SequenceCallback,this);
  status_sub = n.subscribe("vehicle_status",1, &MessageManager::StatusCallback,this);
  action_sub = n.subscribe("task_action",1, &MessageManager::ActionCallback,this);

	station_pub = n.advertise<std_msgs::String>("station_id",1);
	action_pub = n.advertise<std_msgs::Int32>("action_index",1);
	sequence_pub = n.advertise<std_msgs::String>("task_sequence",1);
	command_pub = n.advertise<std_msgs::String>("action_command",1);


  station_sub = n.subscribe("/to_robot_task",1, &MessageManager::StationCallback,this);
	center_pub = n.advertise<std_msgs::String>("/robot_backend_task",1);
}


MessageManager::~MessageManager() {
}

void MessageManager::Initialization() {
	robot_seq_ = "XXXX";
	robot_act_ = "XX";
	robot_speed_ = 0;
	robot_status_ = -1;
	robot_update_ = {false,false,false,false};

	run_timer_ = ros::Time::now();

 	std::size_t pos = robot_id_.find("_");
	string string_temp = robot_id_.substr(pos+1);
	id_number_ = stoi(string_temp);
}

void MessageManager::Execute() {
	Initialization();
  ros::Rate loop_rate(ROS_RATE_HZ);
  cout << "Message Node For " << robot_id_ << " Started" << endl;
  while (ros::ok()) {
  	ros::spinOnce();
    loop_rate.sleep();
    // UpdateRealtimeInfo();
    // cout << "Callback Flag " << robot_update_ << endl;
    if(robot_update_[3] || (robot_update_[0]&&robot_update_[1]&&robot_update_[2])){
    	// cout << "UpdateTaskInfo" << endl;
    	UpdateTaskInfo();
    }
    
  }

  cout << "Message Node For " << robot_id_ << " Closed" << endl;

}

void MessageManager::UpdateTaskInfo() {
	// robot_update_ = {false,false,false,false};

	int size_head          = 6;
	int size_length        = 2;
	int size_source        = 2;
	int size_message_type  = 2;
	int size_robot_id      = 2;
	int size_system_status = 2;
	int size_sequence      = 4;

	int size_mission_type  = 2;
	int size_status        = 2;
	int size_reserve       = 6;
	int size_end           = 2;


	string data_head          = "GPMAPD";
	string data_length        = "26";
	string data_source        = "00";
	string data_message_type  = "00";
	string data_robot_id      = FillInt2String(id_number_,size_robot_id,1);
	string data_system_status = "01";
	string data_sequence      = robot_seq_;


	string data_mission_type  = robot_act_;
	string data_status        = FillInt2String(robot_status_,size_status,1);
	string data_reserve       = "XXXXXX";
	string data_end           = "HE";

	string output_overall = data_head; 
	output_overall += data_length;
	output_overall += data_source;
	output_overall += data_message_type;
	output_overall += data_robot_id;
	output_overall += data_system_status;
	output_overall += data_sequence;

	output_overall += data_mission_type;
	output_overall += data_status;
	output_overall += data_reserve;
	output_overall += data_end;


	std_msgs::String topic_data;
	topic_data.data = output_overall;
	center_pub.publish(topic_data);


	cout << "================================================" << endl;
	cout << "SS             Task Status Update               " << endl;
	cout << "Overall  : " << output_overall << endl;
	cout << "================================================" << endl;
	// cout << "Run Time : " << ((ros::Time::now() - run_timer_).toSec()) << endl;
	// cout << "Robot ID : " << id_number_ << endl;
	// cout << "Sequence : " << data_sequence << endl;
	// cout << "Mission  : " << ConvertMissionType2English(data_mission_type) << endl;
	// cout << "Status   : " << ConvertMissionStatus2English(data_status) << endl;
}

void MessageManager::UpdateRealtimeInfo() {
	int size_coordinate = 8;
	int size_yaw = 4;

	tf::StampedTransform stampedtransform;
  try {
    listener_map_to_base.lookupTransform("/map", robot_id_ + "/base_link",  
                             ros::Time(0), stampedtransform);
  }
  catch (tf::TransformException ex) {
  	cout << "Unable Get Transform from /map to /" << robot_id_ << "/base_link" << endl;
    return;
  }
  int robot_x_cm = stampedtransform.getOrigin().x() * 100;
  int robot_y_cm = stampedtransform.getOrigin().y() * 100;
  int robot_yaw  = tf::getYaw(stampedtransform.getRotation()) * 180/PI;

  string robot_x_str = FillInt2String(robot_x_cm,size_coordinate,0);
  string robot_y_str = FillInt2String(robot_y_cm,size_coordinate,0);
  string robot_yaw_str = FillInt2String(robot_yaw,size_yaw,0);


  int size_head          = 6;
	int size_length        = 2;
	int size_source        = 2;
	int size_message_type  = 2;
	int size_robot_id      = 2;
	int size_system_status = 2;
	int size_sequence      = 4;

	int size_coordinate_x  = size_coordinate;
	int size_coordinate_y  = size_coordinate;
	int size_coordinate_z  = size_coordinate;
	int size_heading       = size_yaw;
	int size_speed         = 4;
	int size_battery       = 2;
	int size_environment   = 2;
	int size_reserve       = 6;
	int size_end           = 2;

	string data_head          = "GPMAPD";
	string data_length        = "58";
	string data_source        = "00";
	string data_message_type  = "01";
	string data_robot_id      = FillInt2String(id_number_,size_robot_id,1);
	string data_system_status = "01";
	string data_sequence      = robot_seq_;

	string data_x             = robot_x_str;
	string data_y             = robot_y_str;
	string data_z             = "XXXXXXXX";
	string data_yaw           = robot_yaw_str;
	string data_speed         = FillInt2String(robot_speed_,size_speed,0);
	string data_battery       = "99";
	string data_environment   = "00";
	string data_reserve       = "XXXXXX";
	string data_end           = "HE";

	string output_overall = data_head; 
	output_overall += data_length;
	output_overall += data_source;
	output_overall += data_message_type;
	output_overall += data_robot_id;
	output_overall += data_system_status;
	output_overall += data_sequence;

	output_overall += data_x;
	output_overall += data_y;
	output_overall += data_z;
	output_overall += data_yaw;
	output_overall += data_speed;
	output_overall += data_battery;
	output_overall += data_environment;
	output_overall += data_reserve;
	output_overall += data_end;

	std_msgs::String topic_data;
	topic_data.data = output_overall;
	// cout << "Heartbeat Data :" << output_overall << endl;

	// center_pub.publish(topic_data);

	// cout << endl << endl;
	// cout << "================================================" << endl;
	// cout << "                 Heartbeat Update               " << endl;
	// cout << "================================================" << endl;
	// cout << "Overall     : " << output_overall << endl;
	// cout << "Time        : " << (ros::Time::now()).toSec() << endl;
	// cout << "Robot ID    : " << id_number_ << endl;
	// cout << "Sequence    : " << data_sequence << endl;
	// cout << "Coordinate  : ("<< robot_x_cm  << " , " << robot_y_cm << ")"<< endl;
	// cout << "Heading     : " << robot_yaw << endl;
	// cout << "Speed       : " << robot_speed_ << endl;

}



void MessageManager::StationCallback(const std_msgs::String::ConstPtr& Input) {
	static string last_input;
	static string last_sequence;
	string input = Input->data;
	if(input == last_input) return;
	last_input = input;

	int size_head          = 6;
	int size_length        = 2;
	int size_source        = 2;
	int size_robot_id      = 2;
	int size_system_status = 2;
	int size_sequence      = 4;
	int size_mission_type  = 2;
	int size_mission       = 8;
	int size_reserve       = 6;
	int size_end           = 2;


	int check_dig = size_head + size_length + size_source;
	string check_id = input.substr(check_dig,size_robot_id);
	if(check_id.at(0) == '0' ){
		check_id = check_id.substr(1.1);
	}
	if(check_id != to_string(id_number_)) return;

	// cout << endl;
	cout << "================================================" << endl;
	cout << "RR              Command Received                " << endl;
	cout << "Overall  : " << input << endl;
	cout << "================================================" << endl;

	string data_head = "GPMAPD";
	string data_source = "01";
	string data_system_status = "01";
	string data_end = "HE";

	int tracking_index = 0;


	if(input.substr(tracking_index,size_head) != data_head) {
		cout << "HEAD Not Match" << endl;
		return;
	}
	tracking_index += size_head;


	if(input.size() - size_head != stoi(input.substr(tracking_index,size_length))) {
		cout << "Size Not Match" << endl;
		return;
	}
	tracking_index += size_length;


	if(input.substr(tracking_index,size_source) != data_source) {
		cout << "Source Unknown" << endl;
		return;
	}
	tracking_index += size_source;


	string robot_id = input.substr(tracking_index,size_robot_id);
	if(robot_id.at(0) == '0' ){
		robot_id = robot_id.substr(1.1);
	}
	tracking_index += size_robot_id;


	if(input.substr(tracking_index,size_system_status) != data_system_status) {
		cout << "System Status Abnormal" << endl;
		return;
	}
	tracking_index += size_system_status;

	
	string task_sequence = input.substr(tracking_index,size_sequence);
	if(last_sequence == task_sequence) {
		cout << "Duplicated Sequence Number" << endl;
		return;
	}
	last_sequence = task_sequence;
	tracking_index += size_sequence;


	string misstion_type = input.substr(tracking_index,size_mission_type);
	tracking_index += size_mission_type;

	string mission_command = input.substr(tracking_index,size_mission);
	tracking_index += size_mission;
	tracking_index += size_reserve;

	if(input.substr(tracking_index,size_end) != data_end) {
		cout << "END Not Match" << endl;
		return;
	}
	robot_update_ = {false,false,false};

	// cout << "Run Time : " << ((ros::Time::now() - run_timer_).toSec()) << endl;
	// cout << "Robot ID : " << id_number_ << endl;
	// cout << "Sequence : " << task_sequence << endl;
	// cout << "Mission  : " << ConvertMissionType2English(misstion_type) << endl;
	if(!ProcessMission(mission_command,misstion_type,task_sequence)) return;
}


int MessageManager::FindMissionType(string Input) {
	if(Input == "00" || Input == "01" || Input == "02" || Input == "08" ) return 1;
	if(Input == "06" || Input == "07") return 0;
	if(Input == "03" || Input == "04" || Input == "05") return 3;
	return -1;
}

bool MessageManager::ProcessMission(string Command,string Type,string Seq) {
	int Action = FindMissionType(Type);
	if(Action < 0) {
		// cout << "Action Unknown" << endl;
		return false;
	} 

	ros::Rate loop_rate(ROS_RATE_HZ);

	string topic_data;
	if(Action == 1) {
		string area_id = Command.substr(0,2);
		string building_number = Command.substr(2,2);
		string gate_number = Command.substr(6,2);
		topic_data = area_id + "-" + building_number + "-" + gate_number;

		std_msgs::String std_string;
		std_string.data = topic_data;
		station_pub.publish(std_string);
		// cout << "Moving to " <<  topic_data << endl;
	}

	// if(Action == 0) cout << "Waiting" << endl;
	// if(Action >= 3) cout << "Action" << endl;

	std_msgs::Int32 action_int;
	action_int.data = Action;
	action_pub.publish(action_int);

	std_msgs::String sequence_string;
	sequence_string.data = Seq;
	sequence_pub.publish(sequence_string);

	std_msgs::String command_string;
	command_string.data = Type;
	command_pub.publish(command_string);



	// if(Action == 1) cout << "Command  : " << topic_data << endl;
	// if(Action == 0) cout << "Command  : Waiting" << endl;
	// if(Action == 3) cout << "Command  : Action" << endl;

}