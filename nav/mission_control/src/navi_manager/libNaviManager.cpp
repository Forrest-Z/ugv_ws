#include "libNaviManager.h"


NaviManager::NaviManager():pn("~"),
isNewGoal_(true),pp_path_index_(0),
isMapSave_(false),isJunSave_(false),
isRecObs_(false),isGoalReached_(true)
{
  pn.param<bool>("use_sim", isUseSim_, true);
  pn.param<string>("junction_file",junction_file_,"Node");


  plan_sub = n.subscribe("/move_base/GlobalPlanner/plan",1, &NaviManager::plan_callback,this);
  vel_sub = n.subscribe("/base_cmd_vel",1, &NaviManager::vel_callback,this);
  goal_sub = n.subscribe("/move_base_simple/goal",1, &NaviManager::goal_callback,this);
  map_sub = n.subscribe("/nav_map",1, &NaviManager::map_callback,this);
  obs_sub = n.subscribe("/clicked_point",1,&NaviManager::obs_callback,this);

  log_pub = n.advertise<std_msgs::String> ("/ugv_log", 1);
  path_pub = n.advertise<sensor_msgs::PointCloud> ("/path_points", 1);
  waypoint_pub = n.advertise<sensor_msgs::PointCloud> ("/waypoint_points", 1);
  vis_pub = n.advertise<visualization_msgs::Marker>( "/waypoint_marker", 0 );
  wall_pub = n.advertise<sensor_msgs::PointCloud> ("/wall_points", 1);
  junction_pub = n.advertise<sensor_msgs::PointCloud> ("/junction_points", 1);
  vel_pub = n.advertise<geometry_msgs::Twist> ("/local_cmd_vel", 1);
  sleep(1);

  paramInitialization();
}


NaviManager::~NaviManager(){
}

void NaviManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  recordLog("Navi_manager Node is Running",LogState::INITIALIZATION);
  while (ros::ok()) {
    NaviManager::Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void NaviManager::paramInitialization() {
  //recordLog("Parameter Initialization Done",LogState::INITIALIZATION);
  robot_position_ = {0,0,0};
  loadJunctionFile(junction_file_);
  if (isUseSim_) {
    recordLog("Runing Simulation Mode",LogState::INITIALIZATION);
  } else {
    recordLog("Show WayPoint and Path Only",LogState::INITIALIZATION);
  }
}

void NaviManager::Mission() {
  simDriving(isUseSim_);
  publishStaticLayer();
  if (!followPurePursuit()) {
    local_cmd_vel_.linear.x = 0;
    local_cmd_vel_.angular.z = 0;
  }

  vel_pub.publish(local_cmd_vel_);
}

void NaviManager::simDriving(bool flag) {
  if (flag) {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robot_position_[0];
    odom_trans.transform.translation.y = robot_position_[1];
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,robot_position_[2]);

    tf_odom.sendTransform(odom_trans);
  }
}


void NaviManager::publishStaticLayer() {
  tf::StampedTransform transform;
  geometry_msgs::Point32 point;
  sensor_msgs::PointCloud pointcloud_map;

  tf::StampedTransform transform_local;
  tf::StampedTransform transform_obs;

  geometry_msgs::PoseStamped map_point;
  geometry_msgs::PoseStamped base_point;

  map_point.header.frame_id = "/map";
  base_point.header.frame_id = "/base_link";

  tf::Stamped<tf::Pose> tf_map;
  tf::Stamped<tf::Pose> tf_base;

  pointcloud_map.header.frame_id = "/base_link";

  try {
    listener.lookupTransform("/map", "/base_link",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    recordLog("Waiting for TF",LogState::WARNNING);
    return;
  }

  vector<double> vehicle_in_map = {transform.getOrigin().x(),transform.getOrigin().y()};
  if(!isUseSim_) { 
    robot_position_ = vehicle_in_map;
    robot_position_[2] = tf::getYaw(transform.getRotation());
  }
  double window_size = 10;

  int search_unit = window_size/static_map_info_.resolution;

  int vehicle_in_grid_x = (vehicle_in_map[0] - static_map_info_.origin.position.x) 
  / static_map_info_.resolution;
  int vehicle_in_grid_y = (vehicle_in_map[1] - static_map_info_.origin.position.y) 
  / static_map_info_.resolution;

  int search_row_begin; 
  int search_row_end;
  int search_col_begin;
  int search_col_end;

  (vehicle_in_grid_x > search_unit) ? search_row_begin = vehicle_in_grid_x - search_unit
  : search_row_begin = 0;
  (vehicle_in_grid_y > search_unit) ? search_col_begin = vehicle_in_grid_y - search_unit 
  : search_col_begin = 0;    

  (vehicle_in_grid_x < static_map_info_.width - search_unit) ? search_row_end = vehicle_in_grid_x + search_unit
  : search_row_end = static_map_info_.width;
  (vehicle_in_grid_y < static_map_info_.height - search_unit) ? search_col_end = vehicle_in_grid_y + search_unit
  : search_col_end = static_map_info_.height;

  for (int i = search_row_begin; i < search_row_end; ++i) {
    for (int j = search_col_begin; j < search_col_end; ++j) {
      if(static_map_.data[i+j*static_map_info_.width] == 0) continue;

      map_point.pose.position.x = i * static_map_info_.resolution + static_map_info_.origin.position.x;
      map_point.pose.position.y = j * static_map_info_.resolution + static_map_info_.origin.position.y;
     
      try {
        listener_local.lookupTransform("/base_link", "/map",  
                         ros::Time(0), transform_local);
      }
      catch (tf::TransformException ex){
        recordLog("Waiting for TF for Wall",LogState::WARNNING);
        return;
      }

      tf::poseStampedMsgToTF(map_point,tf_map);
      listener_local.transformPose("/base_link",ros::Time(0),tf_map,"/map",tf_base);
      tf::poseStampedTFToMsg(tf_base,base_point);

      point.x = base_point.pose.position.x;
      point.y = base_point.pose.position.y;
      point.z = 1;
      pointcloud_map.points.push_back(point);
    }
  }

  if (isUseSim_ && isRecObs_) {
    try {
      listener_obs.lookupTransform("/map", "/base_link",  
                       ros::Time(0), transform_obs);
    }
    catch (tf::TransformException ex){
      recordLog("Waiting for TF for Obstacle",LogState::WARNNING);
      return;
    }

    map_point.pose.position.x = obs_point_.x;
    map_point.pose.position.y = obs_point_.y;

    tf::poseStampedMsgToTF(map_point,tf_map);
    listener_obs.transformPose("/base_link",ros::Time(0),tf_map,"/map",tf_base);
    tf::poseStampedTFToMsg(tf_base,base_point);

    double obs_rad = 0.2;

    for (double i = 0; i < 2 * PI; i+=0.01) {
      geometry_msgs::Point32 point;
      point.x = base_point.pose.position.x+obs_rad+obs_rad*cos(i);
      point.y = base_point.pose.position.y+obs_rad*sin(i);
      point.z = 1;
      pointcloud_map.points.push_back(point);
    }
  }



  wall_pub.publish(pointcloud_map);

  publishJunctionPoints();
}


void NaviManager::recordLog(string input,LogState level) {
  std_msgs::String msg;
  string state;
  string event;
  string color;
  int schedule_selected;
  int log_gap_sec;
  static string last_log = "no data";
  static vector<int> log_schedule_ = {0,0,0,0,0};

  switch (level) {
    case LogState::INITIALIZATION: {
      state = "Initialization";
      event = input;
      schedule_selected = 0;
      log_gap_sec = -1;
      color = WHITE;
    break;
    }
    
    case LogState::INFOMATION: {
      state = "Infomation";
      event = input;
      schedule_selected = 0;
      log_gap_sec = -1;
      color = WHITE;
    break;
    }

    case LogState::STATE_REPORT: {
      state = "Controller State";
      event = input;
      schedule_selected = 1;
      log_gap_sec = 1;
      color = GREEN;
    break;
    }

    case LogState::WARNNING: {
      state = "WARNNING";
      event = input;
      schedule_selected = 2;
      log_gap_sec = 1;
      color = YELLOW;
    break;
    }

    case LogState::ERROR: {
      state = "ERROR";
      event = input;
      schedule_selected = 3;
      log_gap_sec = -1;
      color = RED;
    break;
    }

    default:
      assert(false);
  }


  if (event != last_log || level != LogState::STATE_REPORT) {
    if (log_schedule_[schedule_selected] < int(ros::Time::now().toSec()) - log_gap_sec) {
      msg.data = input;
      //ROS_INFO_STREAM(input);

      time_t clock = time(NULL);
      cout<<color<<"Node  | "<<ros::this_node::getName()<<endl;
      cout<<color<<"Data  | "<<asctime(std::localtime(&clock));
      cout<<color<<"State | "<<state<<endl;
      cout<<color<<"Event | "<<event<<endl;
      cout<<ORIGIN<<endl;
      if (level == LogState::STATE_REPORT) last_log = event;
      log_pub.publish(msg);
      log_schedule_[schedule_selected] = int(ros::Time::now().toSec());
    }
  }
}


void NaviManager::loadJunctionFile(string filename) {
  ifstream my_file;
  my_file.open(filename);
  if (!my_file) {
    recordLog("Unabel to Locate Junction File from " + filename,LogState::WARNNING);
    return;
  }

  geometry_msgs::Point32 point;
  junction_list_.points.clear();
  int line_num = 1;
  while (my_file >> point.z >> point.x >> point.y) {
    junction_list_.points.push_back(point);
    line_num++;
  }
  recordLog("Junction Points Saved, include " + to_string(line_num) 
    + " points",LogState::INITIALIZATION);
  isJunSave_ = true;
}


void NaviManager::publishJunctionPoints() {
  if (!isJunSave_) {
    recordLog("Unabel to Load Junction Point",LogState::WARNNING);
    return;
  }
  junction_list_.points.clear();
  junction_list_.header.frame_id = "map";
  junction_list_.points.push_back(next_goal_);
  junction_pub.publish(junction_list_);
}

void NaviManager::findPurePursuitGoal() {
  double lookahead_distance = 5;
  double goal_distance_from_robot;


  if (global_path_.poses.size() <= 0) return;

  for (int i = pp_path_index_; i < global_path_.poses.size(); i++)
  {
    goal_distance_from_robot = hypot((global_path_.poses[i].pose.position.x - robot_position_[0]),
      (global_path_.poses[i].pose.position.y - robot_position_[1]));
    if (goal_distance_from_robot < lookahead_distance) continue;
    next_goal_.x = global_path_.poses[i].pose.position.x;
    next_goal_.y = global_path_.poses[i].pose.position.y;
    pp_path_index_ = i;
    return;
  }

  next_goal_.x = global_path_.poses[global_path_.poses.size()-1].pose.position.x;
  next_goal_.y = global_path_.poses[global_path_.poses.size()-1].pose.position.y;
  
}

bool NaviManager::followPurePursuit() {
  double goal_tolerance = 0.5;
  double rotation_threshold = PI;
  double speed_scale = 0.3;
  double rotation_scale = 1;

  findPurePursuitGoal();
  if (isGoalReached_) {
    recordLog("Waiting for Plan",LogState::STATE_REPORT);
    return false;
  }

  if (hypot(next_goal_.x - robot_position_[0],
    next_goal_.y - robot_position_[1]) < goal_tolerance) {
    // recordLog("Pure Pursuit Goal Reached",LogState::STATE_REPORT);
    global_path_.poses.clear();
    isGoalReached_ = true;
    return false;
  }

  recordLog("Using Pure Pursuit",LogState::STATE_REPORT);
  double goal_distance = hypot(next_goal_.x - robot_position_[0],
    next_goal_.y - robot_position_[1]);
  double goal_yaw = atan2(next_goal_.y - robot_position_[1],
    next_goal_.x - robot_position_[0]);

  // cout << "robot x y yaw = (" <<  robot_position_[0] << "," << 
  //   robot_position_[1] << "," <<  robot_position_[2] << ")" << endl;
  // cout << "goal  x y yaw = (" << next_goal_.x << "," << next_goal_.y <<
  //   "," << goal_yaw <<")" << endl;


  double temp_index = 0;
  if (goal_yaw < 0) temp_index = 2;
  if (robot_position_[2] < 0) temp_index = -2;

  double rotation = goal_yaw  - robot_position_[2] + temp_index*PI;
  if (rotation < -PI) rotation += 2*PI;
  else if (rotation > PI) rotation -= 2*PI;
  // cout << "yaw different raw = " << rotation << endl;

  if (fabs(rotation) > rotation_threshold) {
    rotation = 0.6;
    goal_distance = 0.3;
  }

  // cout << "yaw different after = " << rotation << endl;


  local_cmd_vel_.linear.x =  speed_scale * goal_distance;
  local_cmd_vel_.angular.z = rotation_scale * rotation;

  if (local_cmd_vel_.linear.x > 1) local_cmd_vel_.linear.x = 1;



  return true;


}