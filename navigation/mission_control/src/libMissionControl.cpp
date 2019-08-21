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
  map_number_pub = n.advertise<std_msgs::Int32>("/map_number",1);
  junction_pub = n.advertise<sensor_msgs::PointCloud>("/junction_point",1);

  clean_path_arrow = n.advertise<visualization_msgs::Marker>("/clean_path",1);

  Initialization();
}
MissionControl::~MissionControl(){
}


void MissionControl::Initialization() {
	map_number_ = 1;

  vehicle_radius_ = 0.6;
  lookahead_time_ = 3;

	max_speed_ = 2;
	max_rotation_ = 1.5;

	max_acc_speed_ = 0.025;
	max_acc_rotation_ = 0.1;

	oscillation_ = 0.02;

  isJoy_ = false;
  isPatrol_ = false;

  last_command_.linear.x = 0;
  last_command_.angular.z = 0;

  isJunSave_ = false;

  loadJunctionFile();
  initPatrol();

}



void MissionControl::Execute() {
	ros::Rate loop_rate(ROS_RATE_HZ);
	Planning MyPlanner(0.5,1);
	ros::Time last_timer = ros::Time::now();
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();

    // if(!checkAllStateReady()) continue;
		sensor_msgs::PointCloud Cloud_Out;

    if(isJoy_){
	    geometry_msgs::Twist test_cmd;

	    test_cmd.linear.x = joy_speed_;
	    test_cmd.linear.z = sp_cmd_;
	    test_cmd.angular.z = joy_rotation_;
	    speedLimit(test_cmd);
	    pathPredict(test_cmd,map_obs_,Cloud_Out);

	    test_cmd.linear.x = joy_speed_;
	    test_cmd.linear.z = sp_cmd_;
	    test_cmd.angular.z = joy_rotation_;
	    speedLimit(test_cmd);
	    speedSmoother(test_cmd);
	    obs_pub.publish(Cloud_Out);
	    vel_pub.publish(test_cmd);

	    tf::StampedTransform transform;
			try {
		    listener.lookupTransform("/map", "/base_link",  
		                             ros::Time(0), transform);
		  }
		  catch (tf::TransformException ex) {
		  	cout << "Waiting For TF NAVI" << endl;
		    continue;
		  }

		  vehicle_in_map_.x = transform.getOrigin().x();
		  vehicle_in_map_.y = transform.getOrigin().y();
		  vehicle_in_map_.z = tf::getYaw(transform.getRotation());

    } else {

    	tf::StampedTransform transform;
			try {
		    listener.lookupTransform("/map", "/base_link",  
		                             ros::Time(0), transform);
		  }
		  catch (tf::TransformException ex) {
		  	cout << "Waiting For TF NAVI" << endl;
		    continue;
		  }

		  vehicle_in_map_.x = transform.getOrigin().x();
		  vehicle_in_map_.y = transform.getOrigin().y();
		  vehicle_in_map_.z = tf::getYaw(transform.getRotation());

	    if(global_path_.poses.size() == 0) continue;

	    double replann_timer = 1;
	    if( (ros::Time::now()-last_timer).toSec() > replann_timer) {
	    	makeGlobalPath(goal_in_map_);
	    	last_timer = ros::Time::now();
	    }

		  geometry_msgs::Point32 current_goal;
		  geometry_msgs::Twist pp_command;
		  double goal_tolerance = 2;

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
	plan_pub.publish(path);
	plan_point_pub.publish(pointcloud);
}

void MissionControl::pathPredict(geometry_msgs::Twist& Cmd_Vel,sensor_msgs::PointCloud& Cloud_In,sensor_msgs::PointCloud& Cloud_Out) {

	double inflation_rate = 5;
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
	Cmd_Vel.linear.x = smootherLogic(Cmd_Vel.linear.x,last_command_.linear.x,max_acc_speed_);
	// Cmd_Vel.angular.z = smootherLogicAngular(Cmd_Vel.angular.z,last_command_.angular.z,max_acc_speed_);
	last_command_ = Cmd_Vel;
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
	  	ROS_INFO("Enter Junction Zone");
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
  		makeGlobalPath(goal_in_map_);
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
  isJunSave_ = true;
}


int MissionControl::isReadyToChangeMap(double input_x,double input_y) {
	int output = -1;
  if (!isJunSave_) return output;

  double junction_range = 10;

  double min_distance = 30;
  int min_index = -1;
  for (int i = 0; i < junction_list_.points.size(); i++) {
  	double junction_distance = hypot((junction_list_.points[i].x - input_x),
      (junction_list_.points[i].y - input_y));
    if (junction_distance < junction_range && junction_distance < min_distance ) {
    	min_distance = (junction_list_.points[i].x - input_x),
      (junction_list_.points[i].y - input_y);
      min_index = i;
    }
  }
  output = min_index;
  return output;
}
