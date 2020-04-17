#include "libMissionControl.h"

MissionControl::MissionControl():pn("~"){
  pn.param<string>("robot_id", robot_id_, "");

  joy_sub = n.subscribe(robot_id_ + "/joy",1, &MissionControl::JoyCallback,this);
  goal_sub = n.subscribe(robot_id_ + "/goal",1, &MissionControl::GoalCallback,this);
  obstacle_sub = n.subscribe(robot_id_ + "/map_obs_points",1, &MissionControl::ObstacleCallback,this); 
  step_sub = n.subscribe(robot_id_ + "/button",1, &MissionControl::StepNumberCallback,this);
  cmd_sub = n.subscribe(robot_id_ + "/virtual_joystick/cmd_vel",1, &MissionControl::CmdCallback,this);

  station_sub = n.subscribe(robot_id_ + "/station_id",1, &MissionControl::StationCallback,this);
  seq_sub = n.subscribe(robot_id_ + "/task_sequence",1, &MissionControl::SequenceCallback,this);
  action_sub = n.subscribe(robot_id_ + "/action_command",1, &MissionControl::ActionCallback,this);
  index_sub = n.subscribe(robot_id_ + "/action_index",1, &MissionControl::IndexCallback,this);


  cmd_vel_pub = n.advertise<geometry_msgs::Twist> (robot_id_ + "/husky_velocity_controller/cmd_vel", 1);
  path_pred_pub = n.advertise<sensor_msgs::PointCloud>(robot_id_ + "/pred_path",1);

  local_goal_pub = n.advertise<sensor_msgs::PointCloud> (robot_id_ + "/local_goal", 1);
  global_goal_pub = n.advertise<sensor_msgs::PointCloud> (robot_id_ + "/global_goal", 1);

  local_all_pub = n.advertise<sensor_msgs::PointCloud>(robot_id_ + "/local_all",1);
  local_safe_pub = n.advertise<sensor_msgs::PointCloud>(robot_id_ + "/local_safe",1);
  local_best_pub = n.advertise<sensor_msgs::PointCloud>(robot_id_ + "/local_best",1);
  local_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>(robot_id_ + "/local_costmap",1);

  station_points_pub = n.advertise<sensor_msgs::PointCloud>(robot_id_ + "/station_points",1);
  global_path_pub = n.advertise<sensor_msgs::PointCloud>(robot_id_ + "/global_path",1);
  vehicle_model_pub = n.advertise<visualization_msgs::Marker>(robot_id_ + "/vehicle_model",1);

  status_pub = n.advertise<std_msgs::Int32>(robot_id_ + "/vehicle_status",1);
  seq_pub = n.advertise<std_msgs::String>(robot_id_ + "/robot_sequence",1);
  action_pub = n.advertise<std_msgs::String>(robot_id_ + "/task_action",1);

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
  if(string_count >= dir_temp.size() - 5) {
    cout << "Navi Folder Error" << endl;
    cout << dir_temp << endl;
    return false;
  }

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

    wait_obstacle_ = true;
  }

  isManualControl_  = false;
  isJoyControl_ = true;
  isLocationUpdate_ = false;
  isReachCurrentGoal_ = false;
  isPauseAction_ = false;
  isMovingBox_ = false;
  isWaitingStop_ = false;

  map_number_ = 1;
  planner_state_ = 1;
  task_status_last_ = -1;

  MyController_.set_speed_scale(controller_linear_scale_);
  MyController_.set_rotation_scale(controller_rotation_scale_);
  MyPlanner_.set_robot_id(robot_id_);
  MyTools_.set_robot_id(robot_id_);

  ReadStationList();

  return(true);
}

void MissionControl::Execute() {
  if(!Initialization()) return;
  PrintConfig();
  
  ros::Rate loop_rate(ROS_RATE_HZ);

  ros::Time planner_timer = ros::Time::now();
  double planner_duration = 0.1;

  cout << "Navigation Node For " << robot_id_ << " Started" << endl;

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    if(isManualControl_) {
      ApplyJoyControl();
    }
    if(!UpdateVehicleLocation()) continue;
    if(!MyPlanner_.UpdateCostmap(obstacle_in_base_)) continue;

    if(isMovingBox_) {
      if((ros::Time::now() - moving_box_timer_).toSec() > 5){
        std_msgs::Int32 status;
        int status_data = 2;
        task_status_last_ = status_data;
        status.data = status_data;
        status_pub.publish(status);
        isMovingBox_ = false;
      } else {
        // cout << "Transporting Items..." << endl;
      }
    }

    if(isWaitingStop_) {
      if((ros::Time::now() - waiting_stop_timer_).toSec() > 3){
        std_msgs::Int32 status;
        int status_data = 2;
        task_status_last_ = status_data;
        status.data = status_data;
        status_pub.publish(status);
        isWaitingStop_ = false;
      } else {
        // cout << "Stopping The Robot..." << endl;
      }
    }


    if(!isManualControl_ && !isPauseAction_ && global_path_pointcloud_.points.size() > 0) {
      ApplyAutoControl(planner_timer,planner_duration);
      UpdateVehicleStatus();
    }

    local_costmap_pub.publish(MyPlanner_.costmap_local());
  }
  cout << "Navigation Node For " << robot_id_ << " Closed" << endl;

}


// ABCD EFGH IJKL MNOP QRST UVWX YZ
void MissionControl::ApplyAutoControl(ros::Time& Timer,double& Duration_Limit) {
  geometry_msgs::Point32 global_goal;
  geometry_msgs::Point32 global_goal_in_local;
  geometry_msgs::Point32 local_goal;
  geometry_msgs::Twist controller_cmd;
  sensor_msgs::PointCloud trajectory_global;
  geometry_msgs::Point32 destination;

  double min_distance = ComputeMinDistance();

  int search_grid = 5;
  double grid_res = 0.4;
  // if(min_distance > 3) search_grid = 5;


  std::size_t pos = robot_id_.find("_");
  string string_temp = robot_id_.substr(pos+1);
  int id_index = stoi(string_temp);
  int wait_time_unit = 3;
  if(wait_obstacle_) {
    static ros::Time detecting_timer;
    static ros::Time waiting_timer;
    if((ros::Time::now() - waiting_timer).toSec() < wait_time_unit * id_index) {
      planner_state_ = 0;
      detecting_timer = ros::Time::now();
    } 
    else if((ros::Time::now() - detecting_timer).toSec() > 6) {
      for (int i = 0; i < obstacle_in_base_.points.size(); ++i) {
        if(obstacle_in_base_.points[i].z > 5) continue;
        double window_size = MyPlanner_.path_window_radius_standard() + 2;
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
  }
 

  if(planner_state_ == 1) {
    MyPlanner_.set_safe_path_search_grid(search_grid);
    MyPlanner_.set_costmap_resolution(grid_res);

    double lookahead_global_meter = 5;
    int global_goal_index = FindCurrentGoalRoute(global_path_pointcloud_,vehicle_in_map_,lookahead_global_meter);
    global_goal = global_path_pointcloud_.points[global_goal_index];
    if(global_goal.z > 0) {

      // cout << "Corner Points" << endl;
      lookahead_global_meter = 1;
      global_goal_index = FindCurrentGoalRoute(global_path_pointcloud_,vehicle_in_map_,lookahead_global_meter);
      global_goal = global_path_pointcloud_.points[global_goal_index];
    }
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
  static geometry_msgs::Twist last_cmd;
  if(isJoyControl_) last_timer_ = ros::Time::now();

  if((ros::Time::now()- last_timer_ ).toSec() > 0.1) {
    isManualControl_ = false;
    cout << "Controller Time out" << endl;  
    return;
  }
  last_cmd = joy_cmd_;
  geometry_msgs::Twist temp_cmd = joy_cmd_;
  LimitCommand(temp_cmd);
  MyTools_.BuildPredictPath(temp_cmd);
  cmd_vel_pub.publish(MyTools_.cmd_vel());
  path_pred_pub.publish(MyTools_.path_predict());
}

void MissionControl::CheckNavigationState(geometry_msgs::Point32 Goal,geometry_msgs::Twist& Cmd_vel) {
  double reach_goal_distance = 3;
  geometry_msgs::Point32 goal_local;
  ConvertPoint(Goal,goal_local,map_to_base_);

  if(hypot(goal_local.x,goal_local.y) < reach_goal_distance) {
    isReachCurrentGoal_ = true;
    ZeroCommand(Cmd_vel);
    global_path_pointcloud_.points.clear();
    cout << "Goal Reached" << endl;
    UpdateVehicleStatus();
  }
}


void MissionControl::ComputeGlobalPlan(geometry_msgs::Point32& Goal) {
  if(!isLocationUpdate_) return;
  isReachCurrentGoal_ = false;
  string map_folder = workspace_folder_ + "config/map/";
  MyRouter_.RoutingAnalyze(Goal,vehicle_in_map_,map_folder,map_number_);
  global_path_pointcloud_ = MyRouter_.path_pointcloud();
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
  cout << "Wait Obstacle              : " << wait_obstacle_ << endl;
  

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
    else if(yaml_info == "wait_obstacle") {
      string reading_data;
      ss >> reading_data;
      if(reading_data == "true") wait_obstacle_ = true;
      else if(reading_data == "false") wait_obstacle_ = false;
      else info_linenum--;
      info_linenum++;
    }

  }

  yaml_file.close(); 

  if(info_linenum == 10) {
    return true;
  } else {
    cout << "Config File Incomplete, Using Default Value" << endl;
    cout << info_linenum << endl;
    return false;
  }
}

bool MissionControl::ReadStationList() {
  std::ifstream station_file;
  string file_name_node = workspace_folder_+ "/config/cfg/station.txt";
  station_file.open(file_name_node);

  if(!station_file.is_open()) {
    cout<<"Could not find Station File"<<endl;
    cout<<"File Not Exist At: "<<file_name_node<<endl;
    return false;
  }

  StationInfo station_temp;
  station_list_.clear();
  int line_num = 0;
  while (station_file >> station_temp.id >> station_temp.x >> station_temp.y) {
    // cout << "station id read :" << station_temp.id << endl;
    station_list_.push_back(station_temp);
    line_num++;
  }
  station_file.close();
  cout << "Read " << line_num << " optional tasks from file" << endl;
  return true;
}

bool MissionControl::UpdateVehicleLocation() {
  // static tf::StampedTransform stampedtransform;
  try {
    listener_map_to_base.lookupTransform("/map", robot_id_ + "/base_link",  
                             ros::Time(0), stampedtransform);
  }
  catch (tf::TransformException ex) {
    // cout << "Waiting For Transform in MissionControl Node" << endl;
    return false;
  }

  isLocationUpdate_ = true;

  vehicle_in_map_.x = stampedtransform.getOrigin().x();
  vehicle_in_map_.y = stampedtransform.getOrigin().y();
  vehicle_in_map_.z = tf::getYaw(stampedtransform.getRotation());

  geometry_msgs::TransformStamped temp_trans;
  temp_trans.header.stamp = ros::Time::now();
  temp_trans.header.frame_id = "/map";
  temp_trans.child_frame_id = robot_id_ + "/base_link";

  temp_trans.transform.translation.x = stampedtransform.getOrigin().x();
  temp_trans.transform.translation.y = stampedtransform.getOrigin().y();
  temp_trans.transform.translation.z = stampedtransform.getOrigin().z();
  temp_trans.transform.rotation.x = stampedtransform.getRotation().x();
  temp_trans.transform.rotation.y = stampedtransform.getRotation().y();
  temp_trans.transform.rotation.z = stampedtransform.getRotation().z();
  temp_trans.transform.rotation.w = stampedtransform.getRotation().w();
  transformMsgToTF(temp_trans.transform,base_to_map_);
  map_to_base_ = base_to_map_.inverse();

  geometry_msgs::Point32 shift_local;
  geometry_msgs::Point32 shift_baseline;
  ConvertPoint(shift_local,shift_baseline,base_to_map_);
  geometry_msgs::Point32 shift_map;
  shift_local.x = -0.5;
  shift_local.y = -0.3;
  ConvertPoint(shift_local,shift_map,base_to_map_);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = stampedtransform.getOrigin().x() + shift_map.x - shift_baseline.x;
  marker.pose.position.y = stampedtransform.getOrigin().y() + shift_map.y - shift_baseline.y;
  marker.pose.position.z = stampedtransform.getOrigin().z() + shift_map.z - shift_baseline.z;
  marker.pose.orientation.x = stampedtransform.getRotation().x();
  marker.pose.orientation.y = stampedtransform.getRotation().y();
  marker.pose.orientation.z = stampedtransform.getRotation().z();
  marker.pose.orientation.w = stampedtransform.getRotation().w();
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.mesh_resource = "package://config/cfg/model.dae";
  vehicle_model_pub.publish(marker);

  return true;
}

void MissionControl::UpdateVehicleStatus() {
  std_msgs::Int32 status;
  int status_data;
  (isReachCurrentGoal_) ? status_data = 2 : status_data = 1;
  if(task_status_last_ == status_data) return;
  task_status_last_ = status_data;
  status.data = status_data;
  status_pub.publish(status);
}