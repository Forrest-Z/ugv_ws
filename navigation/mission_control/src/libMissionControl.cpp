#include "libMissionControl.h"

MissionControl::MissionControl(){
  isUseSim_ = true;
  string odom_topic;
  (isUseSim_) ? odom_topic = "/odom" : odom_topic = "/ekf_2/odometry/filtered";

	goal_sub = n.subscribe("/goal",1, &MissionControl::goal_callback,this);
  map_number_sub = n.subscribe("/map_number",1, &MissionControl::map_number_callback,this);
  map_obs_sub = n.subscribe("/map_obs_points",1, &MissionControl::map_obs_callback,this); 
  joy_sub = n.subscribe("/joy",1, &MissionControl::joy_callback,this);
  odom_sub = n.subscribe(odom_topic,1, &MissionControl::odom_callback,this);
  sonic_sub = n.subscribe("/sonic_state",1, &MissionControl::sonic_callback,this);



  plan_pub = n.advertise<nav_msgs::Path>("/Astar_plan",1);
  plan_point_pub = n.advertise<sensor_msgs::PointCloud>("/Astar_path",1);
  vel_pub = n.advertise<geometry_msgs::Twist> ("/husky_velocity_controller/cmd_vel", 1);
  path_pred_pub = n.advertise<sensor_msgs::PointCloud>("/pred_path",1);
  obs_pub = n.advertise<sensor_msgs::PointCloud> ("/obs_points", 1);
  obs_map_pub = n.advertise<sensor_msgs::PointCloud> ("/obs_points_map", 1);
  map_number_pub = n.advertise<std_msgs::Int32>("/map_number",1);
  junction_pub = n.advertise<sensor_msgs::PointCloud>("/junction_point",1);

  diagnostic_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostic_log",1);

  clean_path_arrow = n.advertise<visualization_msgs::Marker>("/clean_path",1);

  Initialization();
}
MissionControl::~MissionControl(){
}


void MissionControl::Initialization() {

	string config_folder;
	(isUseSim_) ? config_folder = "/home/ha/Workspace/ugv_ws/src/ugv_ws/config/cfg/" : config_folder = "/home/gp/ha_ws/src/ugv_ws/config/config/cfg/";

	if(!readConfig(config_folder)){
		map_number_ = 1;

	  vehicle_radius_ = 0.7;
	  lookahead_time_ = 3;

		max_speed_ = 1.5;
		max_rotation_ = 1.5;

		max_acc_speed_ = 0.025;
		max_acc_rotation_ = 0.1;

		oscillation_ = 0.02;
	}

	printConfig();

  isJoy_ = false;
  isPatrol_ = false;

  isJunSave_ = false;

	odom_speed_ = 0;
	travel_distance_ = 0;

  loadJunctionFile();
  initPatrol();

}

void MissionControl::Execute() {
	ros::Rate loop_rate(ROS_RATE_HZ);
	Planning MyPlanner(2,2);

	ros::Time last_timer = ros::Time::now();
	ros::Time last_loop = ros::Time::now();
	ros::Time last_log = ros::Time::now();
	double log_duration = 1;
	double goal_tolerance = 2;

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();

		sensor_msgs::PointCloud Cloud_Out;

    if(isJoy_){
	    geometry_msgs::Twist joy_cmd;

	    fillCommand(joy_cmd);
	    speedLimit(joy_cmd);
	    pathPredict(joy_cmd,map_obs_,Cloud_Out);

			fillCommand(joy_cmd);
	    speedLimit(joy_cmd);
	    speedSmoother(joy_cmd);
	    obs_pub.publish(Cloud_Out);
	    vel_pub.publish(joy_cmd);

			if(!updateVehicleInMap()) continue;

    } else {
    	geometry_msgs::Point32 current_goal;
		  geometry_msgs::Twist pp_command;

    	if(!updateVehicleInMap()) continue;
	    if(global_path_.poses.size() == 0) continue;

	    if( (ros::Time::now()-last_timer).toSec() > 5 * log_duration) {
	    	makeGlobalPath(goal_in_map_);
	    	last_timer = ros::Time::now();
	    }
	    if(global_path_.poses.size() == 0) continue;

	    generateSafePath(filled_path_pointcloud_,vehicle_in_map_,obs_in_mapframe_);
		  MyPlanner.findCurrentGoal(global_path_,vehicle_in_map_,current_goal);
		  MyPlanner.computePurePursuitCommand(current_goal,vehicle_in_map_,pp_command);
		 	speedLimit(pp_command);


		  if (hypot(current_goal.x - vehicle_in_map_.x,
	    current_goal.y - vehicle_in_map_.y) < goal_tolerance) {
	    	global_path_.poses.clear();
	    	pp_command.linear.x = 0;
	    	pp_command.angular.z = 0;
	    	if(isPatrol_) patrolIndex_++;
	  	}

	  	if(sonic_state_ == 1){
	  		pp_command.linear.x = 0;
	  		pp_command.angular.z  = 0;
	  	}

	  	pathPredict(pp_command,map_obs_,Cloud_Out);
	  	speedSmoother(pp_command);
	  	obs_pub.publish(Cloud_Out);
	  	vel_pub.publish(pp_command);
    }
    
	  checkMapNumber();

	  travel_distance_ += (ros::Time::now()-last_loop).toSec() * odom_speed_;
	  last_loop = ros::Time::now();
	  if( (ros::Time::now()-last_log).toSec() > log_duration) {
	  	runDiagnostic();
	  	last_log = ros::Time::now();
	  }
  }
  
}

void MissionControl::makeGlobalPath(geometry_msgs::Point32 goal_in_map) {
	Routing MyRouter;
	string map_folder;
	(isUseSim_) ? map_folder = "/home/ha/Workspace/ugv_ws/src/ugv_ws/config/map/" : map_folder = "/home/gp/ha_ws/src/ugv_ws/config/map/";
	std::vector<MapGraph> map;
	std::vector<int> path;
	YamlInfo map_info;

	MyRouter.routingAnalyze(goal_in_map,map_folder,map_number_);
	MyRouter.getPath(map,path,map_info);

	publishPlan(map,path,map_info);

	fillPointCloud(global_path_,filled_path_pointcloud_);
}


void MissionControl::publishPlan(std::vector<MapGraph> Map,std::vector<int> Path,YamlInfo Map_info) {
	sensor_msgs::PointCloud pointcloud;
	pointcloud.header.frame_id = "/map";

	nav_msgs::Path path;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/map";
	path.header.frame_id = "/map";

	for (int i = 0; i < Path.size(); i++) {
		geometry_msgs::Point32 point;

		point.x = (Map[Path[Path.size()-i-1]-1].position.y / Map_info.ratio) * Map_info.resolution 
			+ Map_info.origin.x;
		point.y = (Map_info.height - (Map[Path[Path.size()-i-1]-1].position.x / Map_info.ratio)) * Map_info.resolution 
			+ Map_info.origin.y;
		pointcloud.points.push_back(point);

		pose.header.seq = Path.size()-i-1;
		pose.pose.position.x = point.x;
		pose.pose.position.y = point.y;
		path.poses.push_back(pose);
	}

	global_path_ = path;
	// plan_pub.publish(path);
	// plan_point_pub.publish(pointcloud);
}

void MissionControl::pathPredict(geometry_msgs::Twist& Cmd_Vel,sensor_msgs::PointCloud& Cloud_In,sensor_msgs::PointCloud& Cloud_Out) {

	double inflation_rate = 8;
	Controlling MyController(vehicle_radius_,inflation_rate,lookahead_time_,oscillation_);
	MyController.getMovingState(Cmd_Vel);
	MyController.getPredictPath(Cmd_Vel); 
	MyController.filterPointClouds(Cloud_In,Cmd_Vel);

	if(!MyController.waitObstaclePass(Cmd_Vel)) MyController.computeSafePath(Cmd_Vel);

	speedLimit(Cmd_Vel);
	sensor_msgs::PointCloud path_pointcloud;
	path_pointcloud = MyController.path_predict_;
	path_pointcloud.header.stamp = ros::Time::now();
	path_pointcloud.header.frame_id = "/base_link";
	path_pred_pub.publish(path_pointcloud);

	double arrow_yaw = Cmd_Vel.linear.z * PI/180;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "clean_path_arrow";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,arrow_yaw);
	marker.scale.x = 3 * (Cmd_Vel.linear.x/max_speed_);
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	clean_path_arrow.publish(marker);

	Cloud_Out = MyController.pointcloud_filltered_;
}

void MissionControl::generateSafePath(sensor_msgs::PointCloud& Pointcloud,geometry_msgs::Point32 Robot,sensor_msgs::PointCloud&  Obstalce) {

	bool isModified = false;
	ros::Time clock_1 = ros::Time::now();

	if(Pointcloud.points.size() == 0) return;

	for (int i = 0; i < Pointcloud.points.size(); ++i) {

		double distance_from_robot = 
			hypot((Pointcloud.points[i].x - Robot.x),
      (Pointcloud.points[i].y - Robot.y));

		if(distance_from_robot > 10) continue;

		if(Pointcloud.points[i].z == 5) continue;
		double min_x_left = 999;
		double min_x_right = 999;
		double min_id_left = 0;
		double min_id_right = 0;
		double min_y_left = 999;
		double min_y_right = 999;

		double x_local = Pointcloud.points[i].x * cos(Pointcloud.points[i].z) + Pointcloud.points[i].y * sin(Pointcloud.points[i].z);
		double y_local = Pointcloud.points[i].y * cos(Pointcloud.points[i].z) - Pointcloud.points[i].x * sin(Pointcloud.points[i].z);

		for (int j = 0; j < Obstalce.points.size(); ++j) {
			if(Obstalce.points[i].z != 10) continue;
			double x_obs = Obstalce.points[j].x * cos(Pointcloud.points[i].z) + Obstalce.points[j].y * sin(Pointcloud.points[i].z);
			double y_obs = Obstalce.points[j].y * cos(Pointcloud.points[i].z) - Obstalce.points[j].x * sin(Pointcloud.points[i].z);			

			double diff_to_wall_x = x_local - x_obs;
			double diff_to_wall_y = y_local - y_obs;

			if(fabs(diff_to_wall_y) < 5){

				if(diff_to_wall_y > 0) {
					if(fabs(min_x_left) > fabs(diff_to_wall_x)){
						min_x_left = diff_to_wall_x;
						min_id_left = j;
						min_y_left = diff_to_wall_y;
					}
				} 
				else if (diff_to_wall_y < 0) {
					if(fabs(min_x_right) > fabs(diff_to_wall_x)){
						min_x_right = diff_to_wall_x;
						min_id_right = j;
						min_y_right = diff_to_wall_y;
					}				
				}

			}
		}

		if(min_id_left != 0 && min_id_right != 0 && fabs(min_x_right) < 0.1 && fabs(min_x_left) < 0.1) {
			double mid_x = (Obstalce.points[min_id_right].x + Obstalce.points[min_id_left].x)/2;
			double mid_y = (Obstalce.points[min_id_right].y + Obstalce.points[min_id_left].y)/2;
			double point_diff = hypot((mid_x - Pointcloud.points[i].x),(mid_y - Pointcloud.points[i].y));
			if(point_diff > 3) continue;

			// cout << endl;
			// cout << "DEBUG DEBUG" << endl;
			// cout << "point_diff  : " << point_diff << endl; 
			// cout << "min_x_right : " << min_x_right << endl; 
			// cout << "min_y_right : " << min_y_right << endl; 
			// cout << "min_x_left  : " << min_x_left << endl; 
			// cout << "min_y_left  : " << min_y_left << endl; 


			isModified = true;
			Pointcloud.points[i].x = mid_x;
			Pointcloud.points[i].y = mid_y;
			Pointcloud.points[i].z = 5;
		}
	}
	if(isModified) {
		nav_msgs::Path path;
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "/map";
		path.header.frame_id = "/map";

		for (int i = 0; i < Pointcloud.points.size(); ++i) {
			pose.header.seq = i;
			pose.pose.position.x = Pointcloud.points[i].x;
			pose.pose.position.y = Pointcloud.points[i].y;
			path.poses.push_back(pose);
		}

		global_path_ = path;

		plan_point_pub.publish(Pointcloud);
		plan_pub.publish(global_path_);

		isModified = false; 
	}

	// cout<< "compute time cost: " << (ros::Time::now() - clock_1).toSec() << endl;
	return;

	/*
	double modify_range = 30;
	
	ros::Time clock_1 = ros::Time::now();
	int points_ct = 0;
	for (int i = 0; i < Pointcloud.points.size(); ++i) {
		double distance_from_robot = 
    hypot((Pointcloud.points[i].x - Robot.x),
      (Pointcloud.points[i].y - Robot.y));

		for (int j = 0; j < Obstalce.points.size(); ++j) {
			if(Obstalce.points[j].z == 10) continue;
			double diff_x = Pointcloud.points[i].x - Obstalce.points[j].x;
			double diff_y = Pointcloud.points[i].y - Obstalce.points[j].y;			
			double distance_to_path = hypot(diff_x,diff_y);
    	if(distance_to_path < vehicle_radius_ * 3) {
    		geometry_msgs::Point32 point_diff;
    		movePoint(Pointcloud.points[i],point_diff);
    		Pointcloud.points[i].x += 0.8 * point_diff.x;
    		Pointcloud.points[i].y += 0.8 * point_diff.y;
    		Pointcloud.points[i].z = 2;
    		plan_point_pub.publish(Pointcloud);
    		sleep(1);
    		isModified = true;
    	}
		}
		Pointcloud.points[i].z = 1;
	}

	if(isModified) {
		nav_msgs::Path path;
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "/map";
		path.header.frame_id = "/map";

		for (int i = 0; i < Pointcloud.points.size(); ++i) {
			pose.header.seq = i;
			pose.pose.position.x = Pointcloud.points[i].x;
			pose.pose.position.y = Pointcloud.points[i].y;
			path.poses.push_back(pose);
		}

		global_path_ = path;

		plan_point_pub.publish(Pointcloud);
		plan_pub.publish(global_path_);

		isModified = false; 
	}
	*/

}

void MissionControl::movePoint(geometry_msgs::Point32& Input,geometry_msgs::Point32& Output) {
	geometry_msgs::Point32 point_local;
	point_local.x = Input.x * cos(Input.z) - Input.y * sin(Input.z);
	point_local.y = Input.x * sin(Input.z) + Input.y * cos(Input.z);
	point_local.z = Input.z; 

	geometry_msgs::Point32 point_map;
	point_map.x = point_local.x * cos(point_local.z);
	point_map.y = point_local.x * sin(point_local.z);

	Output.x = point_map.x - Input.x;
	Output.y = point_map.y - Input.y;

	cout << "Test movePoint" << endl;
	cout << "Input: " << Input.x << ","<< Input.y<< endl;
	cout << "Local: " << point_local.x << ","<< point_local.y<< endl;
	cout << "Map: " << point_map.x << ","<< point_map.y<< endl;
	cout << "Diff: " << Output.x << ","<< Output.y<< endl;
	cout << "Angle: " << Input.z << endl;
}



void MissionControl::fillPointCloud(nav_msgs::Path& Path, sensor_msgs::PointCloud& Pointcloud) {
	sensor_msgs::PointCloud output_pointcloud;
	sensor_msgs::PointCloud temp_pointcloud;
	output_pointcloud.header.frame_id = "/map";
	temp_pointcloud.header.frame_id = "/map";

	for (int i = 0; i < Path.poses.size(); ++i) {
    geometry_msgs::Point32 point;
    point.x = Path.poses[i].pose.position.x;
    point.y = Path.poses[i].pose.position.y;
    temp_pointcloud.points.push_back(point);
	}


	double points_gap = 1;
	int points_number = 0;

	if(temp_pointcloud.points.size() <= 1) return;
	for (int i = 1; i < temp_pointcloud.points.size(); ++i){
		double diff_x = temp_pointcloud.points[i].x - temp_pointcloud.points[i-1].x;
		double diff_y = temp_pointcloud.points[i].y - temp_pointcloud.points[i-1].y;
		double distance = hypot(diff_x,diff_y);
    points_number = distance / points_gap;
		double points_angle = atan2(diff_y,diff_x);

		for (int j = 0; j < points_number; ++j){
			geometry_msgs::Point32 point;
			point.x = temp_pointcloud.points[i-1].x + j * points_gap * cos(points_angle);
	    point.y = temp_pointcloud.points[i-1].y + j * points_gap * sin(points_angle);
	    point.z = points_angle;
	    // point.z = i;
	    output_pointcloud.points.push_back(point);
		}
	}


	nav_msgs::Path path;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/map";
	path.header.frame_id = "/map";

	for (int i = 0; i < output_pointcloud.points.size(); ++i) {

		pose.header.seq = i;
		pose.pose.position.x = output_pointcloud.points[i].x;
		pose.pose.position.y = output_pointcloud.points[i].y;

		path.poses.push_back(pose);
	}

	Path = path;

	Pointcloud = output_pointcloud;
	plan_point_pub.publish(output_pointcloud);
}


void MissionControl::speedLimit(geometry_msgs::Twist& Cmd_Vel) {
	if(Cmd_Vel.linear.y == 0) {
		if(fabs(Cmd_Vel.linear.x) > max_speed_) {
			Cmd_Vel.linear.x = max_speed_ * (Cmd_Vel.linear.x / fabs(Cmd_Vel.linear.x));
		}
	} else {
		if(fabs(Cmd_Vel.linear.x) > fabs(Cmd_Vel.linear.y) * max_speed_) {
			Cmd_Vel.linear.x = fabs(Cmd_Vel.linear.y) * max_speed_ * (Cmd_Vel.linear.x / fabs(Cmd_Vel.linear.x));
		}
	}

	if(fabs(Cmd_Vel.angular.z) > max_rotation_) {
		Cmd_Vel.angular.z = max_rotation_ * (Cmd_Vel.angular.z / fabs(Cmd_Vel.angular.z));
	}		
	
	if(fabs(Cmd_Vel.angular.z) < oscillation_) {
		Cmd_Vel.angular.z = 0;
	}
}

void MissionControl::speedSmoother(geometry_msgs::Twist& Cmd_Vel) {
	static geometry_msgs::Twist last_command;
	Cmd_Vel.linear.x = smootherLogic(Cmd_Vel.linear.x,last_command.linear.x,max_acc_speed_);
	// Cmd_Vel.angular.z = smootherLogicAngular(Cmd_Vel.angular.z,last_command_.angular.z,max_acc_speed_);
	last_command = Cmd_Vel;
}

double MissionControl::smootherLogic(double cmd,double last,double acc){
	double output = 0;

	if((cmd <= 0 && last > 0) || (cmd >= 0 && last < 0)) output = 0;
	else if(fabs(cmd) - fabs(last) < 2 * acc) output = cmd;
	else if(cmd > last) output = last + acc;
	else if(cmd < last) output = last - acc;

	return output;
}

double MissionControl::smootherLogicAngular(double cmd,double last,double acc){
	double output = 0;

	if((cmd <= 0 && last > 0) || (cmd >= 0 && last < 0)) output = 0;
	else if(fabs(cmd) - fabs(last) < 2 * acc) output = cmd;
	else if(cmd > last) output = last + acc;
	else if(cmd < last) output = last - acc;

	return output;
}

void MissionControl::runPatrolMission(int case_num) {
	static int last_case = 0;
	isPatrol_ = true;

	if(case_num != last_case){
		last_case = case_num;
		patrolIndex_ = 0;
	}
	switch(case_num){
		case 1: {
			if(patrolIndex_>patrolA_.size()-1) patrolIndex_ = 0;
			makeGlobalPath(patrolA_[patrolIndex_]);
		}
		break;

		case 2: {
			if(patrolIndex_>patrolB_.size()-1) patrolIndex_ = 0;
			makeGlobalPath(patrolB_[patrolIndex_]);
		}
		break;

		default: {
			cout << "Unknow Case Num" << endl;
		}
		break;
	}
}

void MissionControl::checkMapNumber() {
  static std_msgs::Int32 map_number;
  int robot_region = map_number_;
  static bool first_enter = false;
  static geometry_msgs::Point32 vehicle_enter;
  static double min_distance;
  static double max_angle;
  static ros::Time last_plan = ros::Time::now();

  junction_list_.header.frame_id = "/map";
  junction_pub.publish(junction_list_);

  int robot_in_junction = isReadyToChangeMap(vehicle_in_map_.x,vehicle_in_map_.y);
  if(robot_in_junction == -1) {
  	map_number.data = robot_region;
  	first_enter = true;
  	return;
  } else {
	  if(first_enter){
	  	vehicle_enter = vehicle_in_map_;
	  	first_enter = false;
	  	min_distance = 100;
	  	max_angle = 0;
	  	cout << "Enter Junction Zone" << endl;
	  }
	  double distance_vehicle_to_points_enter = 
	  		hypot((junction_list_.points[robot_in_junction].x - vehicle_enter.x),
      		(junction_list_.points[robot_in_junction].y - vehicle_enter.y));
	  double distance_vehicle_to_points_now = 
	  		hypot((junction_list_.points[robot_in_junction].x - vehicle_in_map_.x),
      		(junction_list_.points[robot_in_junction].y - vehicle_in_map_.y));
	  double distance_vehicle_moved = 
	  		hypot((vehicle_enter.x - vehicle_in_map_.x),
      		(vehicle_enter.y - vehicle_in_map_.y));

	  double moving_angle = acos( 
	  	(pow(distance_vehicle_to_points_enter,2) + pow(distance_vehicle_to_points_now,2) - pow(distance_vehicle_moved,2)) 
	  	/ (2 * distance_vehicle_to_points_enter * distance_vehicle_to_points_now) 
	  	);

		if(distance_vehicle_to_points_now < min_distance) min_distance = distance_vehicle_to_points_now;
		if(moving_angle > max_angle) max_angle = moving_angle;
	 	int map_1 = int(junction_list_.points[robot_in_junction].z)/10;
	  int map_2 = int(junction_list_.points[robot_in_junction].z)%10;
	  int next_map;

	  if ( max_angle > 1.4 && min_distance < 5) {
	  	(robot_region == map_1) ? next_map = map_2 : next_map = map_1;
	  	robot_region = next_map;
	  	map_number.data = next_map;
	  	first_enter = true;
  		map_number_pub.publish(map_number);
  		if(goal_in_map_.x != 0 && goal_in_map_.y != 0) makeGlobalPath(goal_in_map_);
  		cout << "Change Map" << endl;
	  }

	  if((ros::Time::now()-last_plan).toSec() > 1){
  		if(goal_in_map_.x != 0 && goal_in_map_.y != 0) makeGlobalPath(goal_in_map_);
  		last_plan = ros::Time::now();
	  }



	  // ROS_INFO_STREAM("max_angle    = " << max_angle);
	  // ROS_INFO_STREAM("min_distance = " << min_distance);

  }

}

void MissionControl::loadJunctionFile() {
	string filename;
	(isUseSim_) ? filename = "/home/ha/Workspace/ugv_ws/src/ugv_ws/config/map/junction.txt" : filename = "/home/gp/ha_ws/src/ugv_ws/config/map/junction.txt";

  ifstream my_file;
  my_file.open(filename);
  if (!my_file) {
    cout << "Unabel to Locate Junction File from " << filename << endl;
    return;
  }
  geometry_msgs::Point32 point;
  junction_list_.points.clear();
  int line_num = 0;
  while (my_file >> point.z >> point.x >> point.y) {
    junction_list_.points.push_back(point);
    line_num++;
  }
  (line_num == 0) ? isJunSave_ = false : isJunSave_ = true;
}


int MissionControl::isReadyToChangeMap(double input_x,double input_y) {
	int output = -1;
  if (!isJunSave_) return output;

  double junction_range = 10;
  double min_distance = 999;

  int min_index = -1;
  for (int i = 0; i < junction_list_.points.size(); i++) {
  	double junction_distance = 
  		hypot((junction_list_.points[i].x - input_x),
      			(junction_list_.points[i].y - input_y));
    if (junction_distance < min_distance ) {
    	min_distance = junction_distance;
      min_index = i;
    }
  }

  double min_junction_distance = 
  		hypot((junction_list_.points[min_index].x - input_x),
      			(junction_list_.points[min_index].y - input_y));

  distance_to_junciton_ = min_junction_distance;
  if(min_junction_distance < junction_range) output = min_index;
  
  return output;
}


bool MissionControl::updateVehicleInMap() {
	tf::StampedTransform stampedtransform;
  try {
    listener.lookupTransform("/map", "/base_link",  
                             ros::Time(0), stampedtransform);
  }
  catch (tf::TransformException ex) {
    cout << "Waiting For TF NAVI" << endl;
    return false;
  }

  vehicle_in_map_.x = stampedtransform.getOrigin().x();
  vehicle_in_map_.y = stampedtransform.getOrigin().y();
  vehicle_in_map_.z = tf::getYaw(stampedtransform.getRotation());

  geometry_msgs::TransformStamped temp_trans;
  temp_trans.header.stamp = ros::Time::now();
  temp_trans.header.frame_id = "/map";
  temp_trans.child_frame_id = "/base_link";

  temp_trans.transform.translation.x = stampedtransform.getOrigin().x();
  temp_trans.transform.translation.y = stampedtransform.getOrigin().y();
  temp_trans.transform.translation.z = stampedtransform.getOrigin().z();
  temp_trans.transform.rotation.x = stampedtransform.getRotation().x();
  temp_trans.transform.rotation.y = stampedtransform.getRotation().y();
  temp_trans.transform.rotation.z = stampedtransform.getRotation().z();
  temp_trans.transform.rotation.w = stampedtransform.getRotation().w();
  transformMsgToTF(temp_trans.transform,map_to_base_);

  return true;
}

void MissionControl::fillCommand(geometry_msgs::Twist& Cmd_Vel){
	Cmd_Vel.linear.x = joy_speed_;
	Cmd_Vel.linear.z = sp_cmd_;
	Cmd_Vel.angular.z = joy_rotation_;
}

double MissionControl::evaluatePointCloud(sensor_msgs::PointCloud Input) {
	double output = 0;

	for (int i = 0; i < Input.points.size(); ++i) {
		if(Input.points[i].z == 10) continue;
		double distance = hypot(Input.points[i].x,Input.points[i].y);
		output += 1/distance;
	}

	return output;
}


void MissionControl::runDiagnostic() {
	diagnostic_msgs::DiagnosticArray temp_array;
	diagnostic_msgs::DiagnosticStatus temp_status;
	diagnostic_msgs::KeyValue temp_key;

	temp_array.header.stamp = ros::Time::now();
	temp_array.header.frame_id = "UGV STATUS";

	temp_status.values.clear();
	temp_status.level = 2;
	temp_status.hardware_id = "UGV 001";
	temp_status.name = "Status";
	temp_status.message = "Environment and Driving Status";

	temp_key.key = "Navigation Mode";
	(isJoy_) ? temp_key.value = "Manual" : temp_key.value = "Automatic";
	temp_status.values.push_back(temp_key);

	temp_key.key = "Planning Status";
	(global_path_.poses.size() == 0) ? temp_key.value = "No Plan" : temp_key.value = "Plan Received";
	temp_status.values.push_back(temp_key);

	vector<double> grade_level = {50,100,200};
	string obstalce_state;
	if(obstalce_evaluation_ < grade_level[0]) {
		obstalce_state = "Clean Path";
	}
	else if(obstalce_evaluation_ < grade_level[1]) {
		obstalce_state = "Obstalce Detected";
	}
	else if(obstalce_evaluation_ < grade_level[2]) {
		obstalce_state = "Obstalce Around";
	} else {
		obstalce_state = "Blocked";
	}
	temp_key.key = "Environment Status";
	temp_key.value = obstalce_state;
	temp_status.values.push_back(temp_key);

	temp_array.status.push_back(temp_status);



	temp_status.values.clear();
	temp_status.level = 1;
	temp_status.hardware_id = "UGV 001";
	temp_status.name = "Vehicle Info";
	temp_status.message = "Vehicle Information";

	temp_key.key = "Vehicle Speed";
	temp_key.value = roundString(odom_speed_,2) + "m/s";
	temp_status.values.push_back(temp_key);

	temp_key.key = "Travel Distance";
	temp_key.value = roundString(travel_distance_,2) + "m";
	temp_status.values.push_back(temp_key);

	temp_array.status.push_back(temp_status);



	temp_status.values.clear();
	temp_status.level = 0;
	temp_status.hardware_id = "UGV 001";
	temp_status.name = "Map Info";
	temp_status.message = "Map Information";

	temp_key.key = "Map ID";
	temp_key.value = "MAP-00"+to_string(map_number_);
	temp_status.values.push_back(temp_key);

	temp_key.key = "Closet Junction Point";
	temp_key.value = roundString(distance_to_junciton_,2) + "m";
	temp_status.values.push_back(temp_key);

	temp_array.status.push_back(temp_status);



	diagnostic_pub.publish(temp_array);

}

void MissionControl::convertPointCloudtoMap(sensor_msgs::PointCloud Input,sensor_msgs::PointCloud& Output,tf::Transform Transform) {
	// Transform = Transform.inverse();
	double transform_m[4][4];
  double mv[12];
  Transform.getBasis().getOpenGLSubMatrix(mv);
  tf::Vector3 origin = Transform.getOrigin();

  transform_m[0][0] = mv[0];
  transform_m[0][1] = mv[4];
  transform_m[0][2] = mv[8];
  transform_m[1][0] = mv[1];
  transform_m[1][1] = mv[5];
  transform_m[1][2] = mv[9];
  transform_m[2][0] = mv[2];
  transform_m[2][1] = mv[6];
  transform_m[2][2] = mv[10];

  transform_m[3][0] = transform_m[3][1] = transform_m[3][2] = 0;
  transform_m[0][3] = origin.x();
  transform_m[1][3] = origin.y();
  transform_m[2][3] = origin.z();
  transform_m[3][3] = 1;

  Output = Input;
  Output.header.frame_id = "/map";
  for (int i = 0; i < Input.points.size(); ++i) {
    Output.points[i].x 
      = Input.points[i].x * transform_m[0][0] 
      + Input.points[i].y * transform_m[0][1]
      + Input.points[i].z * transform_m[0][2] 
      + transform_m[0][3];
    Output.points[i].y 
      = Input.points[i].x * transform_m[1][0] 
      + Input.points[i].y * transform_m[1][1] 
      + Input.points[i].z * transform_m[1][2] 
      + transform_m[1][3];   
    Output.points[i].z = Input.points[i].z;
  }
}

bool MissionControl::readConfig(string Folder_dir) {
	std::ifstream yaml_file;
  std::string file_name_node = Folder_dir + "navi_config.txt";
  yaml_file.open(file_name_node);
  if(!yaml_file.is_open()) {
  	cout<<"Could not find Config File"<<endl;
  	cout<<"File Not Exist: "<<file_name_node<<endl;
  	return false;
  }

  string str;
  int info_linenum = 0;
  while (std::getline(yaml_file, str)) {
		std::stringstream ss(str);
		string yaml_info;
		ss >> yaml_info;

		if(yaml_info == "map_number") {	
			ss >> map_number_;
			info_linenum++;
		}
		else if(yaml_info == "vehicle_radius") {
			ss >> vehicle_radius_;
			info_linenum++;
		}
		else if(yaml_info == "lookahead_time") {
			ss >> lookahead_time_;
			info_linenum++;
		}
		else if(yaml_info == "max_speed") {
			ss >> max_speed_;
			info_linenum++;
		}
		else if(yaml_info == "max_rotation") {
			ss >>max_rotation_;
			info_linenum++;
		}
		else if(yaml_info == "max_acc_speed") {
			ss >>max_acc_speed_;
			info_linenum++;
		}
		else if(yaml_info == "max_acc_rotation") {
			ss >>max_acc_rotation_;
			info_linenum++;
		}
		else if(yaml_info == "oscillation") {
			ss >>oscillation_;
			info_linenum++;
		}
	}

	yaml_file.close(); 

	if(info_linenum == 8) {
		return true;
	} else {
		cout << "Config File Incomplete, Using Default Value" << endl;
		return false;
	}
}

void MissionControl::printConfig() {
	cout << "==================================" << endl;
	cout << "           Configuration          " << endl;
	cout << "==================================" << endl;

	cout << "Initial Map Number        : "<< map_number_ << endl;
	cout << "Vehicle Radius            : " << vehicle_radius_ << endl;
	cout << "Lookahead Time            : " << lookahead_time_ << endl;
	cout << "Max Speed                 : " << max_speed_ << endl;
	cout << "Max Rotation              : " << max_rotation_ << endl;
	cout << "Max Acceleration          : " << max_acc_speed_ << endl;
	cout << "Max Rotation Acceleration : " << max_acc_rotation_ << endl;
	cout << "Oscillation               : " << oscillation_ << endl;
	cout << "==================================" << endl;
}