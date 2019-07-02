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



  plan_pub = n.advertise<nav_msgs::Path>("/Astar_plan",1);
  plan_point_pub = n.advertise<sensor_msgs::PointCloud>("/Astar_path",1);
  vel_pub = n.advertise<geometry_msgs::Twist> ("/husky_velocity_controller/cmd_vel", 1);
  path_pred_pub = n.advertise<sensor_msgs::PointCloud>("/pred_path",1);
  obs_pub = n.advertise<sensor_msgs::PointCloud> ("/obs_points", 1);

  clean_path_arrow = n.advertise<visualization_msgs::Marker>("/clean_path",1);

  map_number_ = 1;

  vehicle_radius_ = 0.8;
  lookahead_time_ = 4;


	max_speed_ = 1.2;
	max_rotation_ = 2;

	max_acc_speed_ = 0.015;
	max_acc_rotation_ = 0.1;

	oscillation_ = 0.02;

  isJoy_ = false;
  isPatrol_ = false;

  last_command_.linear.x = 0;
  last_command_.angular.z = 0;

  MissionControl::initPatrol();
}

MissionControl::~MissionControl() {

}

void MissionControl::Execute() {
	ros::Rate loop_rate(ROS_RATE_HZ);
	Planning MyPlanner(0.5,2);
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
/*
		  tf::StampedTransform transform;
			try {
		    listener.lookupTransform("/map", "/base_link",  
		                             ros::Time(0), transform);
		  }
		  catch (tf::TransformException ex) {
		  	cout << "Waiting For TF JOY" << endl;
		    continue;
		  }

		  vehicle_in_map_.x = transform.getOrigin().x();
		  vehicle_in_map_.y = transform.getOrigin().y();
		  vehicle_in_map_.z = tf::getYaw(transform.getRotation());
*/

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
		  geometry_msgs::Point32 current_goal;
		  geometry_msgs::Twist pp_command;
		  double goal_tolerance = 1;

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

	  	pathPredict(pp_command,map_obs_,Cloud_Out);
	  	speedSmoother(pp_command);
	  	obs_pub.publish(Cloud_Out);
	  	vel_pub.publish(pp_command);
    }

    // global_path_.poses.clear();
  }
}

void MissionControl::makeGlobalPath(geometry_msgs::Point32 goal_in_map) {
	Routing MyRouter;
	string map_folder = "/home/gp/ha_ws/src/config/map/"; 
	// /home/gp/ha_ws/src/config/map/  /home/ha/Workspace/test_ws/src/config/map/
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

	MyController.computeSafePath(Cmd_Vel);

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
	if(fabs(Cmd_Vel.linear.x) > max_speed_) {
		Cmd_Vel.linear.x = max_speed_ * (Cmd_Vel.linear.x / fabs(Cmd_Vel.linear.x));
	}
	if(fabs(Cmd_Vel.linear.x)>0.5){
		if(fabs(Cmd_Vel.angular.z) > max_rotation_) {
			Cmd_Vel.angular.z = max_rotation_ * (Cmd_Vel.angular.z / fabs(Cmd_Vel.angular.z));
		}
	} else {
		if(fabs(Cmd_Vel.angular.z) > 2) {
			Cmd_Vel.angular.z = 2 * (Cmd_Vel.angular.z / fabs(Cmd_Vel.angular.z));
		}		
	}
	if(fabs(Cmd_Vel.angular.z) < oscillation_) {
		Cmd_Vel.angular.z = 0;
	}
	
}

void MissionControl::speedSmoother(geometry_msgs::Twist& Cmd_Vel) {

	// if((Cmd_Vel.linear.x < 0 && last_command_.linear.x > 0) || (Cmd_Vel.linear.x > 0 && last_command_.linear.x < 0)) {
	// 	Cmd_Vel.linear.x = 0;
	// }
	// else if(fabs(Cmd_Vel.linear.x) == max_speed_ && fabs(fabs(Cmd_Vel.linear.x)-fabs(last_command_.linear.x)) < 2*max_acc_speed_) {
	// 	Cmd_Vel.linear.x = Cmd_Vel.linear.x;
	// }
	// else if (Cmd_Vel.linear.x > last_command_.linear.x) {
 //    Cmd_Vel.linear.x = last_command_.linear.x + max_acc_speed_;
 //  }
 //  else if (Cmd_Vel.linear.x < last_command_.linear.x) {
 //    Cmd_Vel.linear.x = last_command_.linear.x - max_acc_speed_;
	// }

	// last_command_ = Cmd_Vel;
	Cmd_Vel.linear.x = smootherLogic(Cmd_Vel.linear.x,last_command_.linear.x,max_acc_speed_);
	last_command_ = Cmd_Vel;
}

double MissionControl::smootherLogic(double cmd,double last,double acc){
	double output = 0;

	if((cmd < 0 && last > 0) || (cmd > 0 && last < 0)) output = 0;
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
