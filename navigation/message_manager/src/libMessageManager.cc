#include "libMessageManager.h"

MessageManager::MessageManager():pn("~") {
  n.param<string>("robot_id", robot_id_, "robot_999");
  cmd_sub = n.subscribe("husky_velocity_controller/cmd_vel",10, &MessageManager::CommandCallback,this);
  reset_sub = n.subscribe("/reset_control",10,&MessageManager::ResetCallback,this);
  action_state_sub = n.subscribe("action_state",10, &MessageManager::ActionStateCallback,this);
	
	station_sub = n.subscribe("/to_robot_task",10, &MessageManager::StationCallback,this);
	control_sub = n.subscribe("to_robot_control",10, &MessageManager::ControlCallback,this);
	center_pub = n.advertise<std_msgs::String>("/robot_backend_task",1);
	heart_pub = n.advertise<std_msgs::String>("/robot_backend_info",1);

	command_pub = n.advertise<std_msgs::String>("control_string",1);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1);
	action_pub = n.advertise<std_msgs::Int32>("action_index",1);
}


MessageManager::~MessageManager() {
}

void MessageManager::Initialization() {
	robot_seq_ = "0000";
	robot_act_ = "XX";
	robot_speed_ = 0;

	run_timer_ = ros::Time::now();

	last_input_raw_ = "NEWW";
	last_input_sequence_ = "NEWW";

	last_input_control_ = "NEWW";
	last_input_control_sequence_ = "NEWW";

 	std::size_t pos = robot_id_.find("_");
	string string_temp = robot_id_.substr(pos+1);
	id_number_ = stoi(string_temp);
	cout << "MessageManager Initialized " << robot_id_ << endl;
}

void MessageManager::Execute() {
	Initialization();
  ros::Rate loop_rate(ROS_RATE_HZ);
  cout << "Message Node For " << robot_id_ << " Started" << endl;
  ros::Time heartbeat_timer = ros::Time::now();
  double heartbeat_freq = 1;
  while (ros::ok()) {
  	ros::spinOnce();
    loop_rate.sleep();
    UpdateRealtimeInfo(heartbeat_timer,heartbeat_freq);    
  }

  cout << "Message Node For " << robot_id_ << " Closed" << endl;
}



void MessageManager::UpdateTaskInfo(string Input) {
	int size_robot_id      = 2;

	string data_head          = "GPMAPD";
	string data_length        = "26";
	string data_source        = "00";
	string data_message_type  = "00";
	string data_robot_id      = FillInt2String(id_number_,size_robot_id,1);
	string data_system_status = "01";
	string data_sequence      = robot_seq_;
	string data_mission_type  = robot_act_;
	string data_status        = Input;
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
	// cout << robot_id_ << " UpdateTaskInfo" << endl;
}

void MessageManager::ControlCallback(const std_msgs::String::ConstPtr& Input) {
	string input = Input->data;
	if(input == last_input_control_) return;
	last_input_control_ = input;	
	string input_duplicate = input;

	string delimiter = "$";
	string token;
	size_t pos = 0;

	string msg_head_str;
	int msg_total_size = -1;
	string msg_source_str;
	int msg_id_int = -1;
	string msg_status_str;
	int msg_sequence_int = -1;
	string msg_type_str;
	string msg_task_str;
	string msg_1_str;
	string msg_2_str;
	string msg_end_str;

	string data_head   = "GPMAPD";
	string data_source = "backend";
	string data_status = "ok";
	string data_type   = "basiccontrol";
	string data_master = "masterlock";
	string data_lock   = "deadlock";
	string data_end    = "HE";

	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		msg_head_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Header" << endl;
		return;
	}
	if(msg_head_str != data_head) {
		cout << "HEAD Not Match msg_head/head : " << msg_head_str << "/" << data_head << endl;
		return;
	}


	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		token = input_duplicate.substr(0, pos);
		// cout << "Size token " << token << endl;
		msg_total_size = stoi(token);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Size" << endl;
		return;
	}
	if(msg_total_size != input.size()) {
		cout << "Size Not Match total_size/input_size : " << msg_total_size << "/" << input.size() << endl;
		return;
	}


	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		msg_source_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Source" << endl;
		return;
	}
	if(msg_source_str != data_source) {
		cout << "Source Unknown From : " << msg_source_str << endl;
		return;
	}


	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		token = input_duplicate.substr(0, pos);
		// cout << "ID token " << token << endl;
		msg_id_int = stoi(token);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for ID" << endl;
		return;
	}
	if(msg_id_int != id_number_) return;


	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		msg_status_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Status" << endl;
		return;
	}
	if(msg_status_str != data_status) {
		cout << "System Status : " << msg_status_str << endl;
		return;
	}


	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		token = input_duplicate.substr(0, pos);
		// cout << "Sequence token " << token << endl;
		msg_sequence_int = stoi(token);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Sequence" << endl;
		return;
	}

	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		msg_type_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Message Type" << endl;
		return;
	}
	if(msg_type_str != data_type) {
		cout << "Unknown Message Type : " << msg_type_str << endl;
		return;
	}

	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		msg_task_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Task Type" << endl;
		return;
	}

	if(msg_task_str == data_lock) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Lock Message" << endl;
			return;
		}
	}
	else if (msg_task_str == data_master) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Lock Message" << endl;
			return;
		}
	} else {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Command Message 1" << endl;
			return;
		}
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_2_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Command Message 2" << endl;
			return;
		}
	}

	msg_end_str = input_duplicate; 
	if(msg_end_str != data_end) {
		cout << "END Not Match msg_end/end : " << msg_end_str << "/" << data_end << endl;
		return;
	}


	string output_str;
	output_str = msg_task_str + delimiter + msg_1_str + delimiter + msg_2_str + delimiter;
	std_msgs::String str_pub;
	str_pub.data = output_str;
	command_pub.publish(str_pub);
}

void MessageManager::StationCallback(const std_msgs::String::ConstPtr& Input) {
	string input = Input->data;
	if(input == last_input_raw_) return;
	last_input_raw_ = input;


	int size_head          = 6;
	int size_length        = 2;
	int size_source        = 2;
	int size_robot_id      = 2;
	int size_system_status = 2;

	int size_sequence      = 4;
	int size_mission_type  = 2;
	int size_mission_1     = 8;
	int size_mission_2     = 8;
	int size_mission_3     = 8;
	int size_reserve       = 6;
	int size_end           = 2;


	int total_size = 52;
	if(input.size() != total_size) {
		cout << "Size Not Match total_size/input_size : " << total_size << "/" << input.size() << endl;
		return;
	}

	int check_dig = size_head + size_length + size_source;
	string check_id = input.substr(check_dig,size_robot_id);
	if(check_id.at(0) == '0' ){
		check_id = check_id.substr(1.1);
	}
	if(check_id != to_string(id_number_)) return;


	string data_head = "GPMAPD";
	string data_source = "01";
	string data_system_error_status = "00";
	string data_end = "HE";

	int tracking_index = 0;

	if(input.substr(tracking_index,size_head) != data_head) {
		cout << "HEAD Not Match" << endl;
		return;
	}
	tracking_index += size_head;


	if(input.size() - size_head != stoi(input.substr(tracking_index,size_length))) {
		cout << "Size Byte Mismatch" << endl;
		cout << input << endl;
		return;
	}
	tracking_index += size_length;


	if(input.substr(tracking_index,size_source) != data_source) {
		cout << "Source Unknown" << endl;
		cout << input << endl;
		return;
	}
	tracking_index += size_source;


	string robot_id = input.substr(tracking_index,size_robot_id);
	if(robot_id.at(0) == '0' ){
		robot_id = robot_id.substr(1.1);
	}
	tracking_index += size_robot_id;


	if(input.substr(tracking_index,size_system_status) == data_system_error_status) {
		cout << "System Status Abnormal" << endl;
		cout << input << endl;
		return;
	}
	string system_status = input.substr(tracking_index,size_system_status);
	tracking_index += size_system_status;

	
	string task_sequence = input.substr(tracking_index,size_sequence);
	if(last_input_sequence_ == task_sequence) {
		cout << "Duplicated Sequence Number" << endl;
		cout << input << endl;
		return;
	}
	last_input_sequence_ = task_sequence;
	tracking_index += size_sequence;


	string misstion_type = input.substr(tracking_index,size_mission_type);
	tracking_index += size_mission_type;

	string mission_command_1 = input.substr(tracking_index,size_mission_1);
	tracking_index += size_mission_1;
	string mission_command_2 = input.substr(tracking_index,size_mission_2);
	tracking_index += size_mission_2;

	tracking_index += size_mission_3;
	tracking_index += size_reserve;

	if(input.substr(tracking_index,size_end) != data_end) {
		cout << "END Not Match" << endl;
		cout << input << endl;
		return;
	}
	ProcessMission(misstion_type,task_sequence,mission_command_1,mission_command_2);

	// cout << robot_id_ << " StationCallback" << endl;
}

void MessageManager::ProcessMission(string Type,string Seq,string Command_1,string Command_2) {
	int Action = FindMissionType(Type);
	if(Action < 0) return;
	robot_act_ = Type;
	robot_seq_ = Seq;

	if(Action == 1) {
		geometry_msgs::PoseStamped goal_info;
		goal_info.header.stamp = ros::Time::now();
		goal_info.header.frame_id = "/map";
		int int_sign_x = stoi(Command_1.substr(0,1));
		int x_in_cm = ((int_sign_x * 2) - 1) * stoi(Command_1.substr(1,7));
		int int_sign_y = stoi(Command_2.substr(0,1));
		int y_in_cm = ((int_sign_y * 2) - 1) * stoi(Command_2.substr(1,7));

		goal_info.pose.position.x = static_cast<double>(x_in_cm)/100;
    goal_info.pose.position.y = static_cast<double>(y_in_cm)/100;
    goal_pub.publish(goal_info);
  }
	std_msgs::Int32 action_int;
	action_int.data = stoi(Type);
	action_pub.publish(action_int);
	
	UpdateTaskInfo();
	// cout << robot_id_ << " ProcessMission " << stoi(Type) << endl;
}



void MessageManager::UpdateRealtimeInfo(ros::Time& Timer,double Freq) {

	if((ros::Time::now() - Timer).toSec() < Freq) return;
	Timer = ros::Time::now();

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
	heart_pub.publish(topic_data);
	// cout << robot_id_ << " UpdateRealtimeInfo" << endl;
}

