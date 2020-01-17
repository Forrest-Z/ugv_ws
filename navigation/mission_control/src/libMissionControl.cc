#include "libMissionControl.h"

MissionControl::MissionControl(){
  joy_sub = n.subscribe("/joy",1, &MissionControl::JoyCallback,this);
  goal_sub = n.subscribe("/goal",1, &MissionControl::GoalCallback,this);
  obstacle_sub = n.subscribe("/map_obs_points",1, &MissionControl::ObstacleCallback,this); 
  map_number_sub = n.subscribe("/map_number",1, &MissionControl::MapNumberCallback,this);

  path_pred_pub = n.advertise<sensor_msgs::PointCloud>("/pred_path",1);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist> ("/husky_velocity_controller/cmd_vel", 1);
  map_number_pub = n.advertise<std_msgs::Int32>("/map_number",1);

  local_goal_pub = n.advertise<sensor_msgs::PointCloud> ("/local_goal", 1);
  global_goal_pub = n.advertise<sensor_msgs::PointCloud> ("/global_goal", 1);

  local_all_pub = n.advertise<sensor_msgs::PointCloud>("/local_all",1);
  local_safe_pub = n.advertise<sensor_msgs::PointCloud>("/local_safe",1);
  local_best_pub = n.advertise<sensor_msgs::PointCloud>("/local_best",1);
  local_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_costmap",1);

  junction_points_pub = n.advertise<sensor_msgs::PointCloud>("/junction_points",1);

  global_path_pub = n.advertise<sensor_msgs::PointCloud>("/global_path",1);
}
MissionControl::~MissionControl(){
}


bool MissionControl::Initialization() {
  string dir_temp = GetCurrentWorkingDir();
  int string_count = 0;
  for (string_count; string_count < dir_temp.size() - 5;string_count++) {
    string dir_temp_2 = dir_temp.substr(string_count,6);
    if(dir_temp_2 == "ugv_ws") break;
  }
  if(string_count >= dir_temp.size() - 5) return false;

  string dir_source = dir_temp.substr(0,string_count+6);
  workspace_folder_ = dir_source + "/src/ugv_ws/";

  if(!ReadConfig()) {
    max_linear_velocity_ = 1.5;
    max_rotation_velocity_ = 1.5;
    max_linear_acceleration_ = 1;
    max_rotation_acceleration_ = 1;

    max_translational_velocity_ = 2;
    min_translational_velocity_ = 0.1;

    controller_linear_scale_ = 1;
    controller_rotation_scale_ = 1;

    vehicle_radius_ = 0.3;
  }

  isManualControl_  = false;
  isLocationUpdate_ = false;
  isJunSave_ = false;
  map_number_ = 1;
  planner_state_ = 1;

  MyController_.set_speed_scale(controller_linear_scale_);
  MyController_.set_rotation_scale(controller_rotation_scale_);

  return(LoadJunctionFile());
}

void MissionControl::Execute() {
  if(!Initialization()) return;
  PrintConfig();


  ros::Rate loop_rate(ROS_RATE_HZ);

  ros::Time planner_timer = ros::Time::now();

  double planner_duration = 0.1;

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    if(!UpdateVehicleLocation()) continue;
    if(!MyPlanner_.UpdateCostmap(obstacle_in_base_)) continue;
    if(isManualControl_) {
      ApplyJoyControl();
    } 
    else if(global_path_pointcloud_.points.size() > 0) {
      ApplyAutoControl(planner_timer,planner_duration);
    }
    CheckMapNumber();
    local_costmap_pub.publish(MyPlanner_.costmap_local());
  }  

}




// ABCD EFGH IJKL MNOP QRST UVWX YZ
void MissionControl::ApplyAutoControl(ros::Time& Timer,double& Duration_Limit) {
  geometry_msgs::Point32 global_goal;
  geometry_msgs::Point32 global_goal_in_local;
  geometry_msgs::Point32 local_goal;
  geometry_msgs::Twist controller_cmd;
  sensor_msgs::PointCloud trajectory_global;
  geometry_msgs::Point32 destination;

  static ros::Time detecting_timer;
  static ros::Time waiting_timer;

  double min_distance = ComputeMinDistance();

  int search_grid = 3;
  if(min_distance > 3) search_grid = 5;
 
  if((ros::Time::now() - waiting_timer).toSec() < 3) {
    planner_state_ = 0;
    detecting_timer = ros::Time::now();
  } 
  else if((ros::Time::now() - detecting_timer).toSec() > 6) {
    for (int i = 0; i < obstacle_in_base_.points.size(); ++i) {
      if(obstacle_in_base_.points[i].z > 5) continue;
      double window_size = 5;
      double obstacle_distance = hypot(obstacle_in_base_.points[i].x,obstacle_in_base_.points[i].y);
      double obstacle_edge = hypot(obstacle_in_base_.points[i].x,(obstacle_in_base_.points[i].y - window_size) );
      if(obstacle_in_base_.points[i].x > 0 && obstacle_distance < window_size) {
        double obstacle_cosine = ( pow(window_size,2) + pow(obstacle_distance,2) - pow(obstacle_edge,2) ) / ( 2 * window_size * obstacle_distance );
        double obstacle_angle = acos(obstacle_cosine);
        int obstacle_zone = (obstacle_angle/PI) * 5;
        if(obstacle_zone == 2) {
          waiting_timer = ros::Time::now();
          break;
        }
      }
    }
    planner_state_ = 1; 
  } else {
    planner_state_ = 1; 
  }


  if(planner_state_ == 1) {
    MyPlanner_.set_safe_path_search_grid(search_grid);

    double lookahead_global_meter = 3;
    int global_goal_index = FindCurrentGoalRoute(global_path_pointcloud_,vehicle_in_map_,lookahead_global_meter);
    global_goal = global_path_pointcloud_.points[global_goal_index];
    destination = global_path_pointcloud_.points.back();

    ConvertPoint(global_goal,global_goal_in_local,map_to_base_);

    double lookahead_local_scale = 2;
    int path_lookahead_index;
    if(!MyPlanner_.GenerateCandidatePlan(global_goal_in_local,obstacle_in_base_)) {
      path_lookahead_index = 0;
    } else {
      path_lookahead_index = MyPlanner_.path_best().points.size()/lookahead_local_scale;
    }

    
    local_goal = MyPlanner_.path_best().points[path_lookahead_index];
    MyController_.ComputePurePursuitCommand(global_goal_in_local,local_goal,controller_cmd);
  } 
  else if(planner_state_ == 3) {
    controller_cmd.linear.x = -max_linear_velocity_/5;
    controller_cmd.angular.z = 0;
  }
  else if(planner_state_ == 0) {
    ZeroCommand(controller_cmd);
  }

  if(min_distance < 1) controller_cmd.linear.x *= pow(min_distance,2);


  CheckNavigationState(destination,controller_cmd);
  LimitCommand(controller_cmd);
  MyTools_.BuildPredictPath(controller_cmd);


  /** Debug **/
  geometry_msgs::Point32 local_goal_map;
  ConvertPoint(local_goal,local_goal_map,base_to_map_);

  sensor_msgs::PointCloud current_goal;
  current_goal.header.frame_id = "/map";
  current_goal.header.stamp = ros::Time::now();
  local_goal_map.z = 10;
  current_goal.points.push_back(local_goal_map);
  local_goal_pub.publish(current_goal);

  current_goal.points.clear();
  global_goal.z = 10;
  current_goal.points.push_back(global_goal);
  global_goal_pub.publish(current_goal);

  // if(MyPlanner_.path_all_set().size() > 0) local_all_pub.publish(MyTools_.ConvertVectortoPointcloud(MyPlanner_.path_all_set()));
  if(MyPlanner_.path_safe_set().size() > 0) local_safe_pub.publish(MyTools_.ConvertVectortoPointcloud(MyPlanner_.path_safe_set()));
  local_best_pub.publish(MyPlanner_.path_best());
  global_path_pub.publish(global_path_pointcloud_);

  cmd_vel_pub.publish(MyTools_.cmd_vel());
  path_pred_pub.publish(MyTools_.path_predict());  
}

void MissionControl::ApplyJoyControl() {
  LimitCommand(joy_cmd_);
  MyTools_.BuildPredictPath(joy_cmd_);
  cmd_vel_pub.publish(MyTools_.cmd_vel());
  path_pred_pub.publish(MyTools_.path_predict());
}

void MissionControl::CheckNavigationState(geometry_msgs::Point32 Goal,geometry_msgs::Twist& Cmd_vel) {
  double reach_goal_distance = 3;
  geometry_msgs::Point32 goal_local;
  ConvertPoint(Goal,goal_local,map_to_base_);

  if(hypot(goal_local.x,goal_local.y) < reach_goal_distance) {
    ZeroCommand(Cmd_vel);
    global_path_pointcloud_.points.clear();
    cout << "Goal Reached" << endl;
  }
}


void MissionControl::CheckMapNumber() {
  static std_msgs::Int32 map_number;
  int robot_region = map_number_;
  static bool first_enter = false;
  static bool change_map = false;
  static geometry_msgs::Point32 vehicle_enter;
  static double min_distance;
  static double max_angle;
  static ros::Time last_plan = ros::Time::now();

  junction_list_.header.frame_id = "/map";
  junction_points_pub.publish(junction_list_);

  int robot_in_junction = ComputeJunctionDistance(vehicle_in_map_);
  if(robot_in_junction == -1) {
    map_number.data = robot_region;
    first_enter = true;
    return;
  } else {
    if(first_enter) {
      vehicle_enter = vehicle_in_map_;
      change_map = false;
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

    if (max_angle > 1.4 && min_distance < 5 && change_map == false) {
      (robot_region == map_1) ? next_map = map_2 : next_map = map_1;
      robot_region = next_map;
      map_number.data = next_map;
      // first_enter = true;
      change_map = true;
      map_number_ = next_map;
      map_number_pub.publish(map_number);
      if(goal_in_map_.x != 0 && goal_in_map_.y != 0) ComputeGlobalPlan(goal_in_map_);
      cout << "Change Map" << endl;
    }

    if((ros::Time::now()-last_plan).toSec() > 1){
      if(goal_in_map_.x != 0 && goal_in_map_.y != 0) ComputeGlobalPlan(goal_in_map_);
      last_plan = ros::Time::now();
    }

  }

}

void MissionControl::ComputeGlobalPlan(geometry_msgs::Point32& Goal) {
  if(!isLocationUpdate_) return;
  string map_folder = workspace_folder_ + "config/map/";
  MyRouter_.RoutingAnalyze(Goal,vehicle_in_map_,map_folder,map_number_);
  global_path_pointcloud_ = MyRouter_.path_pointcloud();
}

int MissionControl::ComputeJunctionDistance(geometry_msgs::Point32 Input) {
  int output = -1;
  if (!isJunSave_) return output;

  double junction_range = 10;
  double min_distance = 999;

  int min_index = -1;
  for (int i = 0; i < junction_list_.points.size(); i++) {
    double junction_distance = 
      hypot((junction_list_.points[i].x - Input.x),
            (junction_list_.points[i].y - Input.y));
    if (junction_distance < min_distance ) {
      min_distance = junction_distance;
      min_index = i;
    }
  }

  double min_junction_distance = 
      hypot((junction_list_.points[min_index].x - Input.x),
            (junction_list_.points[min_index].y - Input.y));

  if(min_junction_distance < junction_range) output = min_index;
  
  return output;
}

double MissionControl::ComputeMinDistance() {
  double restrict_range = 0.3;
  double obstacle_distance = DBL_MAX;
  double min_distance = DBL_MAX;

  for (int i = 0; i < obstacle_in_base_.points.size(); ++i) {
    obstacle_distance = hypot(obstacle_in_base_.points[i].x,obstacle_in_base_.points[i].y);
    if(obstacle_distance < min_distance) min_distance = obstacle_distance;
  }

  return min_distance;
}

void MissionControl::ConvertPointcloud(sensor_msgs::PointCloud Input,sensor_msgs::PointCloud& Output,tf::Transform Transform) {
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
void MissionControl::ConvertPoint(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output,tf::Transform Transform) {
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

  Output.x 
    = Input.x * transform_m[0][0] 
    + Input.y * transform_m[0][1]
    + transform_m[0][3];
  Output.y 
    = Input.x * transform_m[1][0] 
    + Input.y * transform_m[1][1] 
    + transform_m[1][3];   
  Output.z = Input.z;
}


int MissionControl::FindCurrentGoalRoute(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Robot,double Lookahead) {
  int output = -1;
  double min_value = DBL_MAX;
  double min_index_i = -1;
  double min_index_j = -1;
  // cout << "=== DEBUG ===" << endl;


  for (int i = 0; i < Path.points.size()-1; ++i) {
    double distance_i 
      = hypot((Path.points[i].x - Robot.x),(Path.points[i].y - Robot.y));
    double distance_j 
      = hypot((Path.points[i+1].x - Robot.x),(Path.points[i+1].y - Robot.y));
    double distance_ij 
      = hypot((Path.points[i].x - Path.points[i+1].x)
      ,(Path.points[i].y - Path.points[i+1].y));

    double value_ij = (distance_i + distance_j - distance_ij) / distance_ij;

    if(value_ij < min_value) {
      min_value = value_ij;
      min_index_i = i;
      min_index_j = i + 1;
      if(distance_j < Lookahead && min_index_j < Path.points.size()-1) {
        min_index_j++;
      }
    }
    // cout << "index : " << i;
    // cout << " -- value : " << value_ij << endl;
  }
  if(min_index_i == -1) min_index_i = Path.points.size() - 1;
  if(min_index_j == -1) min_index_j = Path.points.size() - 1;
  // cout << "min_value   : " << min_value << endl;
  // cout << "min_index_i : " << min_index_i << endl;
  // cout << "min_index_j : " << min_index_j << endl;

  output = min_index_j;

  return output;
}

void MissionControl::LimitCommand(geometry_msgs::Twist& Cmd_vel) {
  static geometry_msgs::Twist last_cmd_vel;

  // Velocity Limit
  if(fabs(Cmd_vel.linear.x) > fabs(max_linear_velocity_)) Cmd_vel.linear.x = ComputeSign(Cmd_vel.linear.x) * max_linear_velocity_;
  if(fabs(Cmd_vel.angular.z) > fabs(max_rotation_velocity_)) Cmd_vel.angular.z = ComputeSign(Cmd_vel.angular.z) * max_rotation_velocity_;
  if(fabs(Cmd_vel.angular.z) < min_translational_velocity_) Cmd_vel.angular.z = 0;

  // Acceleration Limit
  if((Cmd_vel.linear.x * last_cmd_vel.linear.x) < 0 || Cmd_vel.linear.x == 0) Cmd_vel.linear.x = 0;
  else if(fabs(Cmd_vel.linear.x) - fabs(last_cmd_vel.linear.x) < max_linear_acceleration_) Cmd_vel.linear.x = Cmd_vel.linear.x;
  else if(Cmd_vel.linear.x > last_cmd_vel.linear.x) Cmd_vel.linear.x = last_cmd_vel.linear.x + max_linear_acceleration_;
  else if(Cmd_vel.linear.x < last_cmd_vel.linear.x) Cmd_vel.linear.x = last_cmd_vel.linear.x - max_linear_acceleration_;

  // Overall Velocity Limit
  // double velocity_rotation_ratio = max_translational_velocity_ / max_linear_velocity_;
  // double max_moving_rotation_velocity = max_rotation_velocity_ * (velocity_rotation_ratio - fabs(Cmd_vel.linear.x / max_linear_velocity_));
  // if(fabs(Cmd_vel.angular.z) > fabs(max_moving_rotation_velocity)) Cmd_vel.angular.z = ComputeSign(Cmd_vel.angular.z) * max_moving_rotation_velocity;

  double velocity_rotation_ratio = max_translational_velocity_ / max_rotation_velocity_;
  double max_moving_rotation_velocity = max_rotation_velocity_ * (velocity_rotation_ratio - fabs(Cmd_vel.angular.z / max_rotation_velocity_));
  if(fabs(Cmd_vel.linear.x) > fabs(max_moving_rotation_velocity)) Cmd_vel.linear.x = ComputeSign(Cmd_vel.linear.x) * max_moving_rotation_velocity;  

  last_cmd_vel = Cmd_vel;
}

bool MissionControl::LoadJunctionFile() {
  string filename = workspace_folder_ + "config/map/junction.txt";

  std::ifstream my_file;
  my_file.open(filename);
  if (!my_file) {
    cout << "Unabel to Locate Junction File from " << filename << endl;
    return false;
  }
  geometry_msgs::Point32 point;
  junction_list_.points.clear();
  int line_num = 0;
  while (my_file >> point.z >> point.x >> point.y) {
    junction_list_.points.push_back(point);
    line_num++;
  }
  (line_num == 0) ? isJunSave_ = false : isJunSave_ = true;
  return true;
}


void MissionControl::PrintConfig() {
  cout << "==================================" << endl;
  cout << "           Configuration          " << endl;
  cout << "==================================" << endl;

  cout << "Max Linear Velocity        : " << max_linear_velocity_ << endl;
  cout << "Max Rotation Velocity      : " << max_rotation_velocity_ << endl;

  cout << "Max Linear Acceleration    : " << max_linear_acceleration_ << endl;
  cout << "Max Rotation Acceleration  : " << max_rotation_acceleration_ << endl;

  cout << "Max Translational Velocity : " << max_translational_velocity_ << endl;
  cout << "Min Translational Velocity : " << min_translational_velocity_ << endl;

  cout << "Controller Linear Scale    : " << controller_linear_scale_ << endl;
  cout << "Controller Rotation Scale  : " << controller_rotation_scale_ << endl;

  cout << "Vehicle Radius             : " << vehicle_radius_ << endl;

  cout << "==================================" << endl;
}


bool MissionControl::ReadConfig() {
  std::ifstream yaml_file;
  std::string file_name_node = workspace_folder_+ "/config/cfg/navi_config.txt";
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

    if(yaml_info == "max_linear_velocity") { 
      ss >> max_linear_velocity_;
      info_linenum++;
    }
    else if(yaml_info == "max_rotation_velocity") {
      ss >> max_rotation_velocity_;
      info_linenum++;
    }
    else if(yaml_info == "max_linear_acceleration") {
      ss >> max_linear_acceleration_;
      info_linenum++;
    }
    else if(yaml_info == "max_rotation_acceleration") {
      ss >> max_rotation_acceleration_;
      info_linenum++;
    }
    else if(yaml_info == "max_translational_velocity") {
      ss >> max_translational_velocity_;
      info_linenum++;
    }
    else if(yaml_info == "min_translational_velocity") {
      ss >> min_translational_velocity_;
      info_linenum++;
    }
    else if(yaml_info == "controller_linear_scale") {
      ss >> controller_linear_scale_;
      info_linenum++;
    }
    else if(yaml_info == "controller_rotation_scale") {
      ss >> controller_rotation_scale_;
      info_linenum++;
    }
    else if(yaml_info == "vehicle_radius") {
      ss >> vehicle_radius_;
      info_linenum++;
    }
  }

  yaml_file.close(); 

  if(info_linenum == 9) {
    return true;
  } else {
    cout << "Config File Incomplete, Using Default Value" << endl;
    cout << info_linenum << endl;
    return false;
  }
}

bool MissionControl::UpdateVehicleLocation() {
  static tf::StampedTransform stampedtransform;
  try {
    listener_map_to_base.lookupTransform("/map", "/base_link",  
                             ros::Time(0), stampedtransform);
  }
  catch (tf::TransformException ex) {
    cout << "Waiting For Transform in MissionControl Node" << endl;
    return false;
  }

  isLocationUpdate_ = true;

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
  transformMsgToTF(temp_trans.transform,base_to_map_);
  map_to_base_ = base_to_map_.inverse();

  return true;
}