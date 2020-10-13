#include "libMessageManager.h"

MessageManager::MessageManager():pn("~") {
  n.param<string>("robot_id", robot_id_, "robot_12");
  n.param<string>("community_id", community_id_, "1");

  cmd_sub = n.subscribe("husky_velocity_controller/cmd_vel",10, &MessageManager::CommandCallback,this);
  reset_sub = n.subscribe("/reset_control",10,&MessageManager::ResetCallback,this);
  action_state_sub = n.subscribe("action_state",10, &MessageManager::ActionStateCallback,this);
	exp_sub = n.subscribe("divers_exception",10, &MessageManager::ExceptionCallback,this);


	station_sub = n.subscribe("to_robot_task",10, &MessageManager::StationCallback,this);
	control_sub = n.subscribe("to_robot_control",10, &MessageManager::ControlCallback,this);

	center_pub = n.advertise<std_msgs::String>("from_robot_task",1);
	heart_pub = n.advertise<std_msgs::String>("from_robot_status",1);
	info_pub = n.advertise<std_msgs::String>("from_robot_info",1);
	exp_pub = n.advertise<std_msgs::String>("from_robot_exception",1);

	command_pub = n.advertise<std_msgs::String>("control_string",1);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1);
	action_pub = n.advertise<std_msgs::Int32>("action_index",1);
}


MessageManager::~MessageManager() {
}

void MessageManager::Initialization() {
	robot_seq_ = "0";
	robot_act_ = "XX";
	robot_speed_ = 0;

	run_timer_ = ros::Time::now();

	last_input_raw_ = "NEW";
	last_input_sequence_ = "NEW";

	task_id_ = "NEW";

	last_input_control_ = "NEW";
	last_input_control_sequence_ = "NEW";

 	std::size_t pos = robot_id_.find("_");
	string string_temp = robot_id_.substr(pos+1);
	id_number_ = stoi(string_temp);

  battery_state_     = "99";
  charge_state_      = "off";
  masterlock_state_  = "off";
  deadlock_state_    = "off";
  environment_state_ = "0";

  exception_list_.clear();

	cout << "MessageManager Initialized " << robot_id_ << endl;
}

void MessageManager::Execute() {
	Initialization();
  ros::Rate loop_rate(ROS_RATE_HZ);
  cout << "Message Node For " << robot_id_ << " Started" << endl;
  ros::Time heartbeat_timer = ros::Time::now();
  ros::Time realtime_timer = ros::Time::now();
  double heartbeat_pause = 1;
  double realtime_pause = 0.1;
  while (ros::ok()) {
  	ros::spinOnce();
    loop_rate.sleep();
    if(UpdateVehicleLocation()) {
	    UpdateHeartbeat(heartbeat_timer,heartbeat_pause);    
	    UpdateRealtimeInfo(realtime_timer,realtime_pause);
    }

  }

  cout << "Message Node For " << robot_id_ << " Closed" << endl;
}

void MessageManager::UpdateException(ExceptionInfo Input,ExceptionInfo& Output) {
	bool isAddNewException = true;
	for (int i = 0; i < exception_list_.size(); ++i) {
		if(exception_list_[i].parts == Input.parts) {
			if((exception_list_[i].state == "ok" || exception_list_[i].state == "start") && (Input.state != "ok" || Input.state != "start")) continue;
			isAddNewException = false;
			exception_list_[i].state = Input.state;
			Output = exception_list_[i];
		}
	}
	if(isAddNewException) {
		exception_list_.push_back(Input);
		Output = Input;
	}
}

void MessageManager::ExceptionCallback(const std_msgs::String::ConstPtr& Input) {
	string input_duplicate = Input->data;
	string delimiter = "$";
	size_t pos = 0;

	ExceptionInfo exp_raw;
	boost::uuids::uuid exp_uuid = boost::uuids::random_generator()();
	exp_raw.id = boost::uuids::to_string(exp_uuid);
	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		exp_raw.module = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Module" << endl;
		return;
	}

	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		exp_raw.parts = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Parts" << endl;
		return;
	}

	if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
		exp_raw.state = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for State" << endl;
		return;
	}

	ExceptionInfo exp_checked;
	UpdateException(exp_raw,exp_checked);

	string current_exp_id = exp_checked.id;
	string current_exp_module = exp_checked.module;
	string current_exp_parts = exp_checked.parts;
	string current_self_check = exp_checked.state;

  string head_str = "GPMAPD";
  string source_str = "robot";
  string community_str = community_id_;
  string id_str = to_string(id_number_);
  string status_str = "ok";
  string time_str = to_string(ros::Time::now().toNSec()).substr(0,13);
  string type_str = "exception";
  string exp_id_str = current_exp_id;
  string exp_mod_str = current_exp_module;
  string exp_parts_str = current_exp_parts;
  string self_check_str = current_self_check;
  string end_str = "HE";

  int length_raw = head_str.size() + source_str.size() + community_str.size() +
                   id_str.size() + status_str.size() +
                   time_str.size() + type_str.size() + 
                   exp_id_str.size() + exp_mod_str.size() +
                   exp_parts_str.size() + self_check_str.size() +
                   end_str.size() + 12 * delimiter.size();

  length_raw += static_cast<int>(log10(static_cast<double>(length_raw)));
  length_raw ++;
	if(to_string(length_raw).size() != to_string(length_raw-1).size()) length_raw++;
  string length_str = to_string(length_raw);

  string output = head_str + delimiter + 
                length_str + delimiter + 
                source_str + delimiter + 
                community_str + delimiter + 
                id_str + delimiter + 
                status_str + delimiter + 
                time_str + delimiter + 
                type_str + delimiter + 
                exp_id_str + delimiter + 
                exp_mod_str + delimiter + 
                exp_parts_str + delimiter +
                self_check_str + delimiter + 
                end_str;

  std_msgs::String topic_data;
	topic_data.data = output;
	exp_pub.publish(topic_data);
}

void MessageManager::UpdateRealtimeInfo(ros::Time& Timer,double Pause) {
	if((ros::Time::now() - Timer).toSec() < Pause) return;
	Timer = ros::Time::now();
  string delimiter = "$";

  string head_str = "GPMAPD";
  string source_str = "robot";
  string community_str = community_id_;
  string id_str = to_string(id_number_);
  string status_str = "ok";
  string time_str = to_string(ros::Time::now().toNSec()).substr(0,13);
  string type_str = "info";
  string task_str = "trajectory";
  string end_str = "HE";
 
  string msg_1_str = to_string(robot_rotation_);
  string msg_2_str = to_string(robot_speed_);
  string msg_3_str = to_string(robot_x_cm_);
  string msg_4_str = to_string(robot_y_cm_);
  string msg_5_str = to_string(robot_yaw_);

  int length_raw = head_str.size() + source_str.size() + community_str.size() +
                   id_str.size() + status_str.size() +
                   time_str.size() + type_str.size() + 
                   task_str.size() + msg_1_str.size() +
                   msg_2_str.size() + msg_3_str.size() +
                   msg_4_str.size() + msg_5_str.size() +
                   end_str.size() + 14 * delimiter.size();
  length_raw += static_cast<int>(log10(static_cast<double>(length_raw)));
  length_raw ++;
	if(to_string(length_raw).size() != to_string(length_raw-1).size()) length_raw++;
  string length_str = to_string(length_raw);

  string output = head_str + delimiter + 
                length_str + delimiter + 
                source_str + delimiter + 
                community_str + delimiter + 
                id_str + delimiter + 
                status_str + delimiter + 
                time_str + delimiter + 
                type_str + delimiter + 
                task_str + delimiter + 
                msg_1_str + delimiter + 
                msg_2_str + delimiter +
                msg_3_str + delimiter + 
                msg_4_str + delimiter + 
                msg_5_str + delimiter + 
                end_str;

  std_msgs::String topic_data;
	topic_data.data = output;
	info_pub.publish(topic_data);

}


void MessageManager::UpdateHeartbeat(ros::Time& Timer,double Pause) {
	if((ros::Time::now() - Timer).toSec() < Pause) return;
	Timer = ros::Time::now();

	string delimiter = "$";

  string head_str = "GPMAPD";
  string source_str = "robot";
  string community_str = community_id_;
  string id_str = to_string(id_number_);
  string status_str = "ok";
  string time_str = to_string(ros::Time::now().toNSec()).substr(0,13);
  string type_str = "status";
  string end_str = "HE";

  string coord_x_str = to_string(robot_x_cm_);
  string coord_y_str = to_string(robot_y_cm_);
  string heading_str = to_string(robot_yaw_);
  string speed_str = to_string(robot_speed_);
  string battery_str = battery_state_;
  string charge_str = charge_state_;
  string masterlock_str = masterlock_state_;
  string deadlock_str = deadlock_state_;
  string environment_str = "0";


  int length_raw = head_str.size() + source_str.size() + community_str.size() +
                   id_str.size() + status_str.size() +
                   time_str.size() + type_str.size() + 
                   coord_x_str.size() + coord_y_str.size() +
                   heading_str.size() + speed_str.size() +
                   battery_str.size() + charge_str.size() +
                   masterlock_str.size() + deadlock_str.size() + 
                   environment_str.size() +
                   end_str.size() + 17 * delimiter.size();
  length_raw += static_cast<int>(log10(static_cast<double>(length_raw)));
  length_raw ++;
	if(to_string(length_raw).size() != to_string(length_raw-1).size()) length_raw++;
  string length_str = to_string(length_raw);

  string output = head_str + delimiter + 
                length_str + delimiter + 
                source_str + delimiter + 
                community_str + delimiter + 
                id_str + delimiter + 
                status_str + delimiter + 
                time_str + delimiter + 
                type_str + delimiter + 
                coord_x_str + delimiter + 
                coord_y_str + delimiter + 
                heading_str + delimiter +
                speed_str + delimiter + 
                battery_str + delimiter + 
                charge_str + delimiter + 
                masterlock_str + delimiter + 
                deadlock_str + delimiter + 
                environment_str + delimiter + 
                end_str;

  std_msgs::String topic_data;
	topic_data.data = output;
	heart_pub.publish(topic_data);
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
	string msg_community_str;
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
	string data_community_id = community_id_;
	string data_status = "ok";
	string data_type   = "basic";
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
		msg_community_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Community ID" << endl;
		return;
	}
	if(msg_community_str != data_community_id) {
		cout << "Community Unknown From : " << msg_community_str << endl;
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
	if(msg_id_int != id_number_) {
		cout << "Robot ID Not Match MyID/Received: " << id_number_ << "/" << msg_id_int << endl;
		return;
	}

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
		msg_sequence_int = 0;
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
			deadlock_state_ = msg_1_str;
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Lock Message" << endl;
			return;
		}
	}
	else if (msg_task_str == data_master) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			masterlock_state_ = msg_1_str;
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Master Lock Message" << endl;
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
	// if(input == last_input_raw_) return;
	// last_input_raw_ = input;

	string input_duplicate = input;

	string delimiter = "$";
	string token;
	size_t pos = 0;

	string msg_head_str;
	int msg_total_size = -1;
	string msg_source_str;
	string msg_community_str;
	int msg_id_int = -1;
	string msg_status_str;
	int msg_sequence_int = -1;
	string msg_type_str;
	string msg_task_str;
	string msg_1_str;
	string msg_2_str;
	string msg_3_str;
	string msg_end_str;

	string data_head   = "GPMAPD";
	string data_source = "backend";
	string data_community_id = community_id_;
	string data_status = "ok";
	string data_type   = "task";
	string data_goto = "goto";
	string data_picking   = "picking";
	string data_loading   = "loading";
	string data_unloading   = "unloading";
	string data_still     = "still";

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
		msg_community_str = input_duplicate.substr(0, pos);
		input_duplicate.erase(0, pos + delimiter.length());
	} else {
		cout << "Couldn't Find Delimiter for Community ID" << endl;
		return;
	}
	if(msg_community_str != data_community_id) {
		cout << "Community Unknown From : " << msg_community_str << endl;
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
	if(msg_id_int != id_number_) {
		cout << "Robot ID Not Match MyID/Received: " << id_number_ << "/" << msg_id_int << endl;
		return;
	}

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
		// msg_sequence_int = stoi(token);
		msg_sequence_int = 0;
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

	if(msg_task_str == data_goto) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Goto Message 1" << endl;
			return;
		}
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_2_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Goto Message 2" << endl;
			return;
		}
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_3_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Goto Message 3" << endl;
			return;
		}
	}
	else if (msg_task_str == data_loading) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Loading Message 1" << endl;
			return;
		}
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_2_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Loading Message 2" << endl;
			return;
		}
	} 
	else if (msg_task_str == data_unloading) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Unloading Message 1" << endl;
			return;
		}
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_2_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Unloading Message 2" << endl;
			return;
		}
	} 
	else if (msg_task_str == data_picking) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Picking" << endl;
			return;
		}
	} 
	else if (msg_task_str == data_still) {
		if ((pos = input_duplicate.find(delimiter)) != std::string::npos) {
			msg_1_str = input_duplicate.substr(0, pos);
			input_duplicate.erase(0, pos + delimiter.length());
		} else {
			cout << "Couldn't Find Delimiter for Still" << endl;
			return;
		}
	} 
	else {
		cout << "Unknown Task type : " << msg_task_str << endl;
		return;
	}

	msg_end_str = input_duplicate; 
	if(msg_end_str != data_end) {
		cout << "END Not Match msg_end/end : " << msg_end_str << "/" << data_end << endl;
		return;
	}

	if(msg_1_str == task_id_) return;
	task_id_ = msg_1_str;

	ProcessMission(msg_task_str,to_string(msg_sequence_int),msg_2_str,msg_3_str);

	// cout << robot_id_ << " StationCallback" << endl;
}

void MessageManager::ProcessMission(string Type,string Seq,string Command_1,string Command_2) {
	int Action = FindMissionType(Type);
	if(Action < 0) return;
	robot_act_ = Type;
	robot_seq_ = Seq;

	if(Action == 2) {
		geometry_msgs::PoseStamped goal_info;
		goal_info.header.stamp = ros::Time::now();
		goal_info.header.frame_id = "/map";
		int x_in_cm = stoi(Command_1);
		int y_in_cm = stoi(Command_2);

		goal_info.pose.position.x = static_cast<double>(x_in_cm)/100;
    goal_info.pose.position.y = static_cast<double>(y_in_cm)/100;
    goal_pub.publish(goal_info);
  }
	std_msgs::Int32 action_int;
	action_int.data = Action;
	action_pub.publish(action_int);
	
	UpdateTaskInfo();
}



void MessageManager::UpdateTaskInfo(string Input) {
	string delimiter = "$";

  string head_str = "GPMAPD";
  string source_str = "robot";
  string community_str = community_id_;
  string id_str = to_string(id_number_);
  string status_str = "ok";
  string time_str = to_string(ros::Time::now().toNSec()).substr(0,13);
  string type_str = "task";
  string task_str = robot_act_;
  string end_str = "HE";
 
  string msg_1_str = Input;

  int length_raw = head_str.size() + source_str.size() + community_str.size() +
                   id_str.size() + status_str.size() +
                   time_str.size() + type_str.size() + 
                   task_str.size() + msg_1_str.size() + task_id_.size() + 
                   end_str.size() + 11 * delimiter.size();
  length_raw += static_cast<int>(log10(static_cast<double>(length_raw)));
  length_raw ++;
	if(to_string(length_raw).size() != to_string(length_raw-1).size()) length_raw++;
  string length_str = to_string(length_raw);

  string output = head_str + delimiter + 
                length_str + delimiter + 
                source_str + delimiter + 
                community_str + delimiter + 
                id_str + delimiter + 
                status_str + delimiter + 
                time_str + delimiter + 
                type_str + delimiter + 
                task_str + delimiter +
                task_id_ + delimiter +
                msg_1_str + delimiter + 
                end_str;

  std_msgs::String topic_data;
	topic_data.data = output;
	center_pub.publish(topic_data);
	// cout << robot_id_ << " UpdateTaskInfo" << endl;
}

bool MessageManager::UpdateVehicleLocation() {
	tf::StampedTransform stampedtransform;
  try {
    listener_map_to_base.lookupTransform("/map", robot_id_ + "/base_link",  
                             ros::Time(0), stampedtransform);
  }
  catch (tf::TransformException ex) {
  	cout << "Unable Get Transform from /map to /" << robot_id_ << "/base_link" << endl;
    return false;
  }
  robot_x_cm_ = stampedtransform.getOrigin().x() * 100;
 	robot_y_cm_ = stampedtransform.getOrigin().y() * 100;
  robot_yaw_  = tf::getYaw(stampedtransform.getRotation()) * 100;
  // cout << robot_id_ << "update x y yaw : " << robot_x_cm_ << "," << robot_y_cm_ << "," << robot_yaw_ << endl;
  return true;
}
