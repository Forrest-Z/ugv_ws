#include "libLogManager.h"

LogManager::LogManager() {
	cout << "Log Node Started" << endl;
  station_sub = n.subscribe("/to_robot_task",1, &LogManager::ReceivedCallback,this);
  center_sub = n.subscribe("/robot_backend_task",1, &LogManager::UploadCallback,this);

  Initialization();

  cout << "============================================================"<< endl;
	cout << "                           Logging                          "<< endl;
	cout << "============================================================"<< endl;
	cout << "   Date   | Seq  |  From    |    To    |  Command  | State" << endl;
}


LogManager::~LogManager() {
	cout << "Log Node Closed" << endl;
}


void LogManager::Initialization() {
	history_size_ = 100;
}

void LogManager::Execute() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  cout << "Log Node Started" << endl;
  ros::Time print_timer = ros::Time::now();
  while (ros::ok()) {
  	ros::spinOnce();
    loop_rate.sleep();
    // if(ReadytoPrint(print_timer,1)) PrintLogList();
  }
  cout << "Log Node Closed" << endl;
}

void LogManager::PrintLogList(LogInfo Input) {
	string separater = " | ";
	cout << " " << Input.date << separater;
	cout << Input.seq << separater;
	cout << Input.from << separater;
	cout << Input.to << separater;
	cout << Input.command << separater;
	cout << Input.state << endl;
}

void LogManager::ReceivedCallback(const std_msgs::String::ConstPtr& Input) {
	LogInfo output;

	string input = Input->data;
	if(CheckDuplicateHistory(input)) return;
	AddHistroy(input);

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

	string data_head          = "GPMAPD";
	string data_source        = "01";
	string data_system_status = "01";
	string data_end = "HE";

	int tracking_index = 0;

	if(input.substr(tracking_index,size_head) != data_head) return;
	tracking_index += size_head;

	if(input.size() - size_head != stoi(input.substr(tracking_index,size_length))) return;
	tracking_index += size_length;

	if(input.substr(tracking_index,size_source) != data_source) return;
	tracking_index += size_source;

	string robot_id = input.substr(tracking_index,size_robot_id);
	tracking_index += size_robot_id;

	if(input.substr(tracking_index,size_system_status) != data_system_status) return;
	tracking_index += size_system_status;

	string task_sequence = input.substr(tracking_index,size_sequence);
	tracking_index += size_sequence;

	string misstion_type = input.substr(tracking_index,size_mission_type);
	tracking_index += size_mission_type;

	string mission_command = input.substr(tracking_index,size_mission);
	tracking_index += size_mission;
	tracking_index += size_reserve;

	if(input.substr(tracking_index,size_end) != data_end) return;

	output.date = SaveCurrentDate();
	output.seq = task_sequence;
	output.from = "backend ";
	output.to = "robot_" + robot_id;
	output.command = ConvertMissionType2English(misstion_type);
	output.state = ConvertMissionDetial2English(mission_command,misstion_type);
	PrintLogList(output);
}

void LogManager::UploadCallback(const std_msgs::String::ConstPtr& Input) {
	LogInfo output;

	string input = Input->data;
	if(CheckDuplicateHistory(input)) return;
	AddHistroy(input);


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
	string data_source        = "00";
	string data_message_type  = "00";
	string data_system_status = "01";

	string data_end = "HE";

	int tracking_index = 0;

	if(input.substr(tracking_index,size_head) != data_head) return;
	tracking_index += size_head;

	if(input.size() - size_head != stoi(input.substr(tracking_index,size_length))) return;
	tracking_index += size_length;

	if(input.substr(tracking_index,size_source) != data_source) return;
	tracking_index += size_source;

	if(input.substr(tracking_index,size_message_type) != data_message_type){
		tracking_index += size_source;
	} else {
		tracking_index += size_source;

		string robot_id = input.substr(tracking_index,size_robot_id);
		tracking_index += size_robot_id;

		if(input.substr(tracking_index,size_system_status) != data_system_status) return;
		tracking_index += size_system_status;

		string task_sequence = input.substr(tracking_index,size_sequence);
		tracking_index += size_sequence;

		string misstion_type = input.substr(tracking_index,size_mission_type);
		tracking_index += size_mission_type;

		string mission_state = input.substr(tracking_index,size_status);
		tracking_index += size_status;
		tracking_index += size_reserve;
		if(input.substr(tracking_index,size_end) != data_end) return;

		output.date = SaveCurrentDate();
		output.seq = task_sequence;
		output.from = "robot_" + robot_id;
		output.to = "backend ";
		output.command = ConvertMissionType2English(misstion_type);
		output.state = ConvertMissionStatus2English(mission_state);
		PrintLogList(output);
	}
}

