#include "libNaviManager.h"


NaviManager::NaviManager():pn("~"),
isNewGoal_(true),
isMapSave_(false),isJunSave_(false),
isRecObs_(false),isGoalReached_(true), 
isRecovery_(false)
{
  pn.param<bool>("use_sim", isUseSim_, true);
  pn.param<string>("junction_file",junction_file_,"Node");
  pn.param<string>("base_frame", base_frame_, "/base_link");
  pn.param<string>("camera_frame", camera_frame_, "/power2_point");
  pn.param<string>("lidar_frame", lidar_frame_, "/rslidar");
  pn.param<string>("map_frame", map_frame_, "/map");
  pn.param<double>("speed_scale", speed_scale_, 0.3);
  pn.param<double>("rotation_scale", rotation_scale_, 0.5);
  pn.param<double>("lookahead_distance", lookahead_distance_, 3);

  plan_sub = n.subscribe("/move_base/GlobalPlanner/plan",1, &NaviManager::plan_callback,this);
  vel_sub = n.subscribe("/base_cmd_vel",1, &NaviManager::vel_callback,this);
  goal_sub = n.subscribe("/navi_goal",1, &NaviManager::goal_callback,this);
  map_sub = n.subscribe("/nav_map",1, &NaviManager::map_callback,this);
  obs_sub = n.subscribe("/clicked_point",1,&NaviManager::obs_callback,this);
  astar_state_sub = n.subscribe("/nav_state",1,&NaviManager::astar_state_callback,this);
  initpose_sub = n.subscribe("/initialpose",1, &NaviManager::initpose_callback,this);

  log_pub = n.advertise<std_msgs::String> ("/ugv_log", 1);
  path_pub = n.advertise<sensor_msgs::PointCloud> ("/path_points", 1);
  waypoint_pub = n.advertise<sensor_msgs::PointCloud> ("/waypoint_points", 1);
  vis_pub = n.advertise<visualization_msgs::Marker>( "/waypoint_marker", 1);
  wall_pub = n.advertise<sensor_msgs::PointCloud> ("/wall_points", 1);
  junction_pub = n.advertise<sensor_msgs::PointCloud> ("/junction_points", 1);
  vel_pub = n.advertise<geometry_msgs::Twist> ("/local_cmd_vel", 1);
  move_base_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  move_base_cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
  call_map_pub = n.advertise<std_msgs::Int32>("/map_number",1);
  navi_state_pub = n.advertise<std_msgs::Int32>("/navi_state",1);
  odom_pub = n.advertise<nav_msgs::Odometry>("/sim_odom",1); 
  pp_pub = n.advertise<sensor_msgs::PointCloud> ("/pp_current_goal", 1);

  sleep(1);

  paramInitialization();
}


NaviManager::~NaviManager(){
}

void NaviManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  recordLog("Navi_manager Node is Running",LogState::INITIALIZATION);
  while (ros::ok()) {
    Mission();
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
  publishJunctionPoints();
  
  publishCurrentGoal();

  // if(findPointFromThreeZone(robot_position_[0],robot_position_[1])==0) {
  //   static int ros_timer = 0;
  //   int loop_s = 5;
  //   if (ros_timer > loop_s * ROS_RATE_HZ) {
  //     isNewGoal_ = true;
  //     ros_timer = 0;
  //   } else {
  //     ros_timer++;
  //   } 
  // }


  // static double last_goal_x = 0;
  // static double last_goal_y = 0;

  // if(last_goal_x!=navi_goal_.x && last_goal_y != navi_goal_.y) {
  //   isNewGoal_ = true;
  //   isMapSave_ = false;
  //   geometry_msgs::PoseStamped msg;
  //   msg.header.frame_id = map_frame_;
  //   msg.header.stamp = ros::Time::now();
  //   msg.pose.position.x = navi_goal_.x;
  //   msg.pose.position.y = navi_goal_.y; 
  //   msg.pose.orientation.w =1.0;
  //   move_base_goal_pub.publish(msg);
  //   last_goal_x = navi_goal_.x;
  //   last_goal_y = navi_goal_.y;
  // }
  


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
    odom_trans.child_frame_id = base_frame_;

    odom_trans.transform.translation.x = robot_position_[0];
    odom_trans.transform.translation.y = robot_position_[1];
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,robot_position_[2]);

    tf_odom.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x = robot_position_[0];
    odom.pose.pose.position.y = robot_position_[1];
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom_pub.publish(odom);

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

  map_point.header.frame_id = map_frame_;
  base_point.header.frame_id = base_frame_;

  tf::Stamped<tf::Pose> tf_map;
  tf::Stamped<tf::Pose> tf_base;

  pointcloud_map.header.frame_id = base_frame_;

  try {
    listener.lookupTransform(map_frame_, base_frame_,  
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
        listener_local.lookupTransform(base_frame_,map_frame_,  
                         ros::Time(0), transform_local);
      }
      catch (tf::TransformException ex){
        recordLog("Waiting for TF for Wall",LogState::WARNNING);
        return;
      }

      tf::poseStampedMsgToTF(map_point,tf_map);
      listener_local.transformPose(base_frame_,ros::Time(0),tf_map,map_frame_,tf_base);
      tf::poseStampedTFToMsg(tf_base,base_point);

      point.x = base_point.pose.position.x;
      point.y = base_point.pose.position.y;
      point.z = 1;
      pointcloud_map.points.push_back(point);
    }
  }

  if (isUseSim_ && isRecObs_) {
    try {
      listener_obs.lookupTransform(map_frame_, base_frame_,  
                       ros::Time(0), transform_obs);
    }
    catch (tf::TransformException ex){
      recordLog("Waiting for TF for Obstacle",LogState::WARNNING);
      return;
    }

    for (int i = 0; i < obs_point_.size(); ++i) {

      map_point.pose.position.x = obs_point_[i].x;
      map_point.pose.position.y = obs_point_[i].y;

      tf::poseStampedMsgToTF(map_point,tf_map);
      listener_obs.transformPose(base_frame_,ros::Time(0),tf_map,map_frame_,tf_base);
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
  }

  wall_pub.publish(pointcloud_map);
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
      msg.data = ros::this_node::getName() + ": " + input;
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
  int line_num = 0;
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

  junction_list_.header.frame_id = map_frame_;
  junction_pub.publish(junction_list_);
}

void NaviManager::findPurePursuitGoal() {
  double goal_distance_from_robot;
  double goal_distance_to_end;
  double robot_distance_to_end;
  double diff_goal_robot;
  int search_index = 10;

  sensor_msgs::PointCloud pp_goal;
  pp_goal.header.frame_id = map_frame_;

  if (global_path_.poses.size() <= 0) return;
  int min_index;
  double min_distance = DBL_MAX;
  for (int i = 0; i < global_path_.poses.size(); i++)
  {
    goal_distance_from_robot = hypot((global_path_.poses[i].pose.position.x - robot_position_[0]),
      (global_path_.poses[i].pose.position.y - robot_position_[1]));
    if (goal_distance_from_robot < min_distance) {
      min_distance = goal_distance_from_robot;
      min_index = i;
    }
  }


  if(min_index == 0) {
    next_goal_.x = global_path_.poses[min_index+1].pose.position.x;
    next_goal_.y = global_path_.poses[min_index+1].pose.position.y;
  }
  else if(min_index >= global_path_.poses.size() - 1) {
    next_goal_.x = global_path_.poses[global_path_.poses.size()-1].pose.position.x;
    next_goal_.y = global_path_.poses[global_path_.poses.size()-1].pose.position.y;
  } else {
    next_goal_.x = global_path_.poses[min_index+1].pose.position.x;
    next_goal_.y = global_path_.poses[min_index+1].pose.position.y;
  }

  pp_goal.points.push_back(next_goal_);
  pp_pub.publish(pp_goal);
}

bool NaviManager::followPurePursuit() {
  double goal_tolerance = 1;
  double rotation_threshold = PI;

  findPurePursuitGoal();
  if (isGoalReached_) {
    std_msgs::Int32 msg;
    msg.data = 1;
    navi_state_pub.publish(msg);
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
  //   robot_position_[1] << "," <<  robot_position_[2] << ")" <<Junction endl;
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
    rotation = 0.8;
    goal_distance = 0.1;
  }
  // cout << "yaw different after = " << rotation << endl;
  local_cmd_vel_.linear.x =  speed_scale_ * goal_distance;
  local_cmd_vel_.angular.z = rotation_scale_ * rotation;

  return true;
}

int NaviManager::findPointFromTwoZone(double input_x,double input_y) {

  // return 1;

  if (!isJunSave_) {
    recordLog("Unabel to Load Junction Point",LogState::WARNNING);
    return -2;
  }
  if (junction_list_.points.size() != 1) return -1;

  double junction_range = 30;

  if (hypot((junction_list_.points[0].x - input_x),
    (junction_list_.points[0].y - input_y)) < junction_range) return 0;
  else if (input_x > junction_list_.points[0].x) return 2;
  else return 1;
}

int NaviManager::findPointFromThreeZone(double input_x,double input_y) {

  // return 1;

  if (!isJunSave_) {
    recordLog("Unabel to Load Junction Point",LogState::WARNNING);
    return -2;
  }
  //if (junction_list_.points.size() != 1) return -1;

  double junction_range = 25; //25

  if (hypot((junction_list_.points[0].x - input_x),
    (junction_list_.points[0].y - input_y)) < junction_range) return 0;
  else if (input_x < junction_list_.points[0].x) return 1;
  else if (input_x > junction_list_.points[0].x && 
    input_y > junction_list_.points[0].y) return 2;
  else return 3;
}

void NaviManager::publishCurrentGoal() {
  static geometry_msgs::Point32 current_goal;
  static std_msgs::Int32 map_number;

  int goal_region = findPointZone(navi_goal_.x,navi_goal_.y); 
  int robot_region = findPointZone(robot_position_[0],robot_position_[1]);
  int changeIndex = goal_region - robot_region;
  int robot_in_junction = isReadyToChangeMap(robot_position_[0],robot_position_[1]);
  int current_map = robot_in_junction/10;
  int next_map = robot_in_junction%10;

  int junction_index = -1;



  if(changeIndex == 0) {
    current_goal = navi_goal_;
    map_number.data = robot_region;
  } 
  else if (abs(changeIndex) == 1)
  { 
    junction_index = findJunctionIndex(goal_region,robot_region);

    if(robot_in_junction == junction_list_.points[junction_index].z) {
      current_goal = navi_goal_;
      map_number.data = goal_region;
    } else {
      current_goal.x = junction_list_.points[junction_index].x;
      current_goal.y = junction_list_.points[junction_index].y;
      map_number.data = robot_region;
    }
  } else {
    if(robot_in_junction > 0) {
      junction_index 
        = findJunctionIndex(robot_region+2*sign(changeIndex),robot_region+1*sign(changeIndex));
      if(robot_in_junction == junction_list_.points[findJunctionIndex(robot_region+1*sign(changeIndex),robot_region)].z){
        map_number.data = robot_region+1*sign(changeIndex);
      } else {
        junction_index = findJunctionIndex(robot_region+1*sign(changeIndex),robot_region);
        map_number.data = robot_region;
      }
    } else {
      junction_index = findJunctionIndex(robot_region+1*sign(changeIndex),robot_region);
      map_number.data = robot_region;
    }
    current_goal.x = junction_list_.points[junction_index].x;
    current_goal.y = junction_list_.points[junction_index].y;
  }

  static double last_goal_x = 0;
  static double last_goal_y = 0;

  // cout<< "DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG" << endl;
  // cout<< "robot_position " << robot_position_[0] << " " << robot_position_[1] << endl;
  // cout<< "navi_goal      " << navi_goal_.x << " " << navi_goal_.y << endl;
  // cout<< "current_goal   " << current_goal.x << " " << current_goal.y << endl;
  // cout<< "last_goal      " <<last_goal_x << " " << last_goal_y << endl;
  // cout<< "goal zone      " <<findPointZone(navi_goal_.x,navi_goal_.y)<<endl;
  // cout<< "robot zone     " <<findPointZone(robot_position_[0],robot_position_[1])<<endl;


  if (!(current_goal.x == last_goal_x && current_goal.y == last_goal_y) 
    && (!isGoalReached_)) {
    isNewGoal_ = true;
    isMapSave_ = false;
    recordLog("New Goal Received",LogState::INFOMATION);
    last_goal_x = current_goal.x;
    last_goal_y = current_goal.y;

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = current_goal.x;
    msg.pose.position.y = current_goal.y; 
    msg.pose.orientation.w =1.0;
    move_base_goal_pub.publish(msg);

    call_map_pub.publish(map_number);
  }

  // if(isRecovery_) {
  //   geometry_msgs::PoseStamped msg;
  //   msg.header.frame_id = map_frame_;
  //   msg.header.stamp = ros::Time::now();
  //   msg.pose.position.x = current_goal.x;
  //   msg.pose.position.y = current_goal.y; 
  //   msg.pose.orientation.w =1.0;
  //   move_base_goal_pub.publish(msg);
  //   isRecovery_ = false;
  // }

}

int NaviManager::findJunctionIndex(int goal,int robot) {
  if (!isJunSave_) {
    recordLog("Unabel to Load Junction Point",LogState::WARNNING);
    return -1;
  }

  for (int i = 0; i < junction_list_.points.size(); i++) {
    int zone_1 = int(junction_list_.points[i].z) / 10;
    int zone_2 = int(junction_list_.points[i].z) % 10;
    if((robot==zone_1&&goal==zone_2)||(robot==zone_2&&goal==zone_1))return i;
  }

  return -1;

}

int NaviManager::findPointZone(double input_x,double input_y) {
  if (!isJunSave_) {
    recordLog("Unabel to Load Junction Point",LogState::WARNNING);
    return -1;
  }
  // if (input_x < junction_list_.points[1].x 
  //   && input_y < junction_list_.points[0].y) return 1;
  // else if (input_x > junction_list_.points[2].x) return 4;
  // else if (input_x < junction_list_.points[1].x && 
  //   input_y > junction_list_.points[0].y) return 2;
  // else return 3;

  if (input_x < junction_list_.points[1].x
    && input_y > junction_list_.points[0].y) return 2;
  else if (input_x > junction_list_.points[3].x) return 5;
  else if (input_x > junction_list_.points[2].x
    && input_x < junction_list_.points[3].x) return 4;
  else if (input_x > junction_list_.points[1].x
    && input_x < junction_list_.points[2].x) return 3;
  else return 1;
}

int NaviManager::isReadyToChangeMap(double input_x,double input_y) {
  if (!isJunSave_) {
    recordLog("Unabel to Load Junction Point",LogState::WARNNING);
    return -1;
  }

  double junction_range = 10;

  for (int i = 0; i < junction_list_.points.size(); i++) {
    if (hypot((junction_list_.points[i].x - input_x),
      (junction_list_.points[i].y - input_y)) < junction_range)
      return junction_list_.points[i].z;
  }

  return -1;
}


