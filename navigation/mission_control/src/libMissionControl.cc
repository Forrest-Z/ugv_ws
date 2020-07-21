#include "libMissionControl.h"

MissionControl::MissionControl():pn("~") {
  pn.param<string>("config_folder", config_folder_, "");
  cout << "config_folder :" << config_folder_ << endl;

  n.param<string>("robot_id", robot_id_, "");

  joy_sub = n.subscribe("joy",1, &MissionControl::JoyCallback,this);
  goal_sub = n.subscribe("goal",1, &MissionControl::GoalCallback,this);
  obstacle_sub = n.subscribe("map_obs_points",1, &MissionControl::ObstacleCallback,this); 
  step_sub = n.subscribe("button",1, &MissionControl::StepNumberCallback,this);
  cmd_sub = n.subscribe("virtual_joystick/cmd_vel",1, &MissionControl::CmdCallback,this);
  tcp_sub = n.subscribe("tcp_string",1, &MissionControl::TcpCallback,this);
  map_number_sub = n.subscribe("map_number",1, &MissionControl::MapNumberCallback,this);

  cmd_vel_pub = n.advertise<geometry_msgs::Twist> ("husky_velocity_controller/cmd_vel", 1);
  path_pred_pub = n.advertise<sensor_msgs::PointCloud>("pred_path",1);

  local_goal_pub = n.advertise<sensor_msgs::PointCloud> ("local_goal", 1);
  global_goal_pub = n.advertise<sensor_msgs::PointCloud> ("global_goal", 1);

  local_all_pub = n.advertise<sensor_msgs::PointCloud>("local_all",1);
  local_safe_pub = n.advertise<sensor_msgs::PointCloud>("local_safe",1);
  local_best_pub = n.advertise<sensor_msgs::PointCloud>("local_best",1);
  local_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("local_costmap",1);

  station_points_pub = n.advertise<sensor_msgs::PointCloud>("station_points",1);
  global_path_pub = n.advertise<sensor_msgs::PointCloud>("global_path",1);
  vehicle_model_pub = n.advertise<visualization_msgs::Marker>("vehicle_model",1);
  vehicle_info_pub = n.advertise<visualization_msgs::Marker>("vehicle_info",1);
  traceback_pub = n.advertise<sensor_msgs::PointCloud>("trace_back",1);
  auto_supervise_state_pub = n.advertise<mission_control::AutoSuperviseState>("auto_supervise_state",1);




  reset_sub = n.subscribe("/reset_control",1,&MissionControl::ResetCallback,this);
  action_sub = n.subscribe("action_index",1,&MissionControl::ActionCallback,this);
  action_state_pub = n.advertise<std_msgs::Int32>("action_state",1);
}
MissionControl::~MissionControl(){
  cout << "Navigation Node For Closed ~~~" << endl;
}


bool MissionControl::Initialization() {
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

  isWIFIControl_ = false;
  isJoyControl_  = false;
  isLTEControl_  = false;

  isAction_ = false;
  isLocationUpdate_ = false;
  isReachCurrentGoal_ = false;

  map_number_ = 1;

  MyController_.set_speed_scale(controller_linear_scale_);
  MyController_.set_rotation_scale(controller_rotation_scale_);

  global_path_pointcloud_.points.clear();

  return(true);
}

void MissionControl::Execute() {
  if(!Initialization()) return;
  PrintConfig();
  
  ros::Rate loop_rate(ROS_RATE_HZ);

  ros::Time planner_timer = ros::Time::now();
  double planner_duration = 0.1;

  cout << "Navigation Node Started" << endl;

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();

    auto mission_state = DecisionMaker();

    if(mission_state == static_cast<int>(AutoState::JOY)) {
      ApplyJoyControl();
      continue;
    }
    else if(mission_state == static_cast<int>(AutoState::LTE)) {
      ApplyLTEControl();
      continue;
    }
    else if(mission_state == static_cast<int>(AutoState::WIFI)) {
      ApplyWIFIControl();
      continue;      
    }
    else if(mission_state == static_cast<int>(AutoState::AUTO)) {
      ApplyAutoControl(mission_state);
      continue;
    }
    else if(mission_state == static_cast<int>(AutoState::SLOW)) {
      ApplyAutoControl(mission_state);//ApplySlowControl();
      continue; 
    }
    else if(mission_state == static_cast<int>(AutoState::STOP)) {
      ApplyStopControl();
      continue; 
    }    
    else if(mission_state == static_cast<int>(AutoState::BACK)) {
      ApplyAutoControl(mission_state); //ApplyBackwardControl();
      continue; 
    } else if(mission_state == static_cast<int>(AutoState::ACTION)) {
      ApplyAction();
      continue; 
    } else if(mission_state == static_cast<int>(AutoState::LOADING)) {
      ApplyLoading();
      continue; 
    }     
    else {
      cout << "Unknown Mission State" << endl;
      continue;
    }
  }
  cout << "Navigation Node For Closed" << endl;

}


// ABCD EFGH IJKL MNOP QRST UVWX YZ
int MissionControl::DecisionMaker() {
  if(isJoyControl_) return static_cast<int>(AutoState::JOY);
  return static_cast<int>(AutoState::LOADING);
  if(isAction_) return static_cast<int>(AutoState::ACTION);
  if(checkMissionStage()) return static_cast<int>(AutoState::AUTO);

  return static_cast<int>(AutoState::STOP);
}

void MissionControl::ApplyAction() {
  double action_time = 2;
  if((ros::Time::now() - action_start_timer_).toSec() > action_time) {
    std_msgs::Int32 action_state;
    action_state.data = current_action_index_;
    action_state_pub.publish(action_state);
    isAction_ = false;
  }
}

void MissionControl::ApplyLoading() {
  
}

void MissionControl::ApplyJoyControl() {
  geometry_msgs::Twist safe_cmd;
  geometry_msgs::Twist raw_cmd = getJoyCommand();
  checkCommandSafety(raw_cmd,safe_cmd);
  createCommandInfo(safe_cmd);
  publishCommand();
  if(!updateState()) return;
  return;
}

void MissionControl::ApplyLTEControl() {
  geometry_msgs::Twist safe_cmd;
  geometry_msgs::Twist raw_cmd = getLTECommand();
  checkCommandSafety(raw_cmd,safe_cmd);
  createCommandInfo(safe_cmd);
  publishCommand();
  if(!updateState()) return;
  return;
}

void MissionControl::ApplyWIFIControl() {
  geometry_msgs::Twist safe_cmd;
  geometry_msgs::Twist raw_cmd = getWIFICommand();
  checkCommandSafety(raw_cmd,safe_cmd);
  createCommandInfo(safe_cmd);
  publishCommand();
  if(!updateState()) return;
  return;
}

void MissionControl::ApplyStopControl() {
  geometry_msgs::Twist safe_cmd;
  geometry_msgs::Twist raw_cmd;
  ZeroCommand(raw_cmd);
  createCommandInfo(safe_cmd);
  publishCommand();
  if(!updateState()) return;
  return;
}

void MissionControl::ApplyAutoControl(int mission_state) {
  geometry_msgs::Twist safe_cmd;
  if(!setAutoCoefficient(1)) return;
  if(!updateState()) return;
  geometry_msgs::Twist raw_cmd = getAutoCommand();
  auto supervise_state = SuperviseDecision(mission_state);
  raw_cmd = ExecuteDecision(raw_cmd,mission_state,supervise_state);
  checkCommandSafety(raw_cmd,safe_cmd);
  createCommandInfo(safe_cmd);
  publishCommand();
  publishInfo();
}

geometry_msgs::Twist MissionControl::getAutoCommand() {
  
  int global_goal_index = FindCurrentGoalRoute(global_path_pointcloud_,vehicle_in_map_,lookahead_global_meter_);
  global_sub_goal_ = global_path_pointcloud_.points[global_goal_index];
  // geometry_msgs::Point32 destination = global_path_pointcloud_.points.back();

  geometry_msgs::Point32 global_goal_in_local;
  ConvertPoint(global_sub_goal_,global_goal_in_local,map_to_base_);

  sensor_msgs::PointCloud temp_pointcloud;
  MyRouter_.ModifyRoutePoint(global_goal_in_local,temp_pointcloud,obstacle_in_base_);
  // local_all_pub.publish(temp_pointcloud);

  if(temp_pointcloud.points.size() > 0) {
    if(temp_pointcloud.points.back().z == -1) {
      global_goal_in_local = temp_pointcloud.points.back();
      // ConvertPoint(global_goal_in_local,global_sub_goal_,base_to_map_);
    }
  }

  int path_lookahead_index;
  geometry_msgs::Twist controller_cmd;

  double search_range = MyPlanner_.map_window_radius();
  double search_range_min = 2;
  double iteration_scale = 0.8;
  double wait_range = 4;
  wait_plan_state_ = true;
  while(search_range > search_range_min) {
    if(!MyPlanner_.GenerateCandidatePlan(global_goal_in_local,obstacle_in_base_,search_range)) {
      if(search_range < wait_range) wait_plan_state_ = false;
    } else {
      break;
    }
    search_range *= iteration_scale;

    if(MyPlanner_.path_all_set().size() > 0) local_all_pub.publish(MyTools_.ConvertVectortoPointcloud(MyPlanner_.path_all_set()));
  }
  

  if(search_range <= search_range_min) {
    plan_state_ = false;
    path_lookahead_index = 0;
  } else {
    path_lookahead_index = MyPlanner_.path_best().points.size()/lookahead_local_scale_;
    plan_state_ = true;
  }

  local_sub_goal_ = MyPlanner_.path_best().points[path_lookahead_index];
  MyController_.ComputePurePursuitCommand(global_goal_in_local,local_sub_goal_,controller_cmd);

  return controller_cmd;
}




bool MissionControl::CheckNavigationState() {
  if(isReachCurrentGoal_) return false;
  double reach_goal_distance = 3;
  geometry_msgs::Point32 goal_local;
  ConvertPoint(goal_in_map_,goal_local,map_to_base_);

  if(hypot(goal_local.x,goal_local.y) < reach_goal_distance) {
    isReachCurrentGoal_ = true;
    global_path_pointcloud_.points.clear();
    return false;
  }
  
  return true;
}


void MissionControl::ComputeGlobalPlan(geometry_msgs::Point32& Goal) {
  if(!isLocationUpdate_) return;
  string map_folder = config_folder_ + "/map/";
  MyRouter_.RoutingAnalyze(Goal,vehicle_in_map_,map_folder,map_number_);
  global_path_pointcloud_ = MyRouter_.path_pointcloud();
  global_path_pointcloud_.points.push_back(goal_in_map_);
  isReachCurrentGoal_ = false;
  // goal_in_map_ = MyRouter_.path_pointcloud().points.back();
  
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

  for (int i = 0; i < Path.points.size()-1; ++i) {
    double lookahead_applied = Lookahead - Lookahead * Path.points[i].z * 0;
    double distance_i 
      = hypot((Path.points[i].x - Robot.x),(Path.points[i].y - Robot.y));
    double distance_j 
      = hypot((Path.points[i+1].x - Robot.x),(Path.points[i+1].y - Robot.y));
    double distance_ij 
      = hypot((Path.points[i].x - Path.points[i+1].x)
      ,(Path.points[i].y - Path.points[i+1].y));
    double curve_ration = Path.points[i].z;

    double value_ij = (distance_i + distance_j - distance_ij) / distance_ij;

    if(value_ij < min_value) {
      min_value = value_ij;
      min_index_i = i;
      min_index_j = i + 1;
      if(distance_j < lookahead_applied && min_index_j < Path.points.size()-1) {
        min_index_j++;
      }
    }
  }
  if(min_index_i == -1) min_index_i = Path.points.size() - 1;
  if(min_index_j == -1) min_index_j = Path.points.size() - 1;

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
  else if(Cmd_vel.linear.x > 0 && Cmd_vel.linear.x < last_cmd_vel.linear.x) {Cmd_vel.linear.x = last_cmd_vel.linear.x - max_linear_acceleration_;}
  else if(fabs(Cmd_vel.linear.x) - fabs(last_cmd_vel.linear.x) < max_linear_acceleration_) {Cmd_vel.linear.x = Cmd_vel.linear.x;}
  else if(Cmd_vel.linear.x > last_cmd_vel.linear.x) Cmd_vel.linear.x = last_cmd_vel.linear.x + max_linear_acceleration_;
  else if(Cmd_vel.linear.x < last_cmd_vel.linear.x) Cmd_vel.linear.x = last_cmd_vel.linear.x - max_linear_acceleration_;

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

  cout << "==================================" << endl;
}


bool MissionControl::ReadConfig() {
  std::ifstream yaml_file;
  std::string file_name_node = config_folder_+ "/cfg/navi_config.txt";
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
  return true;
}

bool MissionControl::UpdateVehicleLocation() {
  try {
    listener_map_to_base.lookupTransform("/map", robot_id_ + "/base_link",  
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
  shift_local.x = -6;
  shift_local.y = -3;
  ConvertPoint(shift_local,shift_map,base_to_map_);

  visualization_msgs::Marker marker;
  visualization_msgs::Marker infoer;
  
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = stampedtransform.getOrigin().x() + shift_map.x - shift_baseline.x;
  marker.pose.position.y = stampedtransform.getOrigin().y() + shift_map.y - shift_baseline.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = stampedtransform.getRotation().x();
  marker.pose.orientation.y = stampedtransform.getRotation().y();
  marker.pose.orientation.z = stampedtransform.getRotation().z();
  marker.pose.orientation.w = stampedtransform.getRotation().w();
  marker.color.a = 1.0;
  marker.color.r = 1;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.scale.x = 6;
  marker.scale.y = 6;
  marker.scale.z = 6;
  marker.mesh_resource = "package://config/cfg/model.dae";


  infoer.header.frame_id = robot_id_ + "/base_link";
  infoer.header.stamp = ros::Time::now();
  infoer.id = 0;
  infoer.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  infoer.action = visualization_msgs::Marker::ADD;
  infoer.pose.position.z = 5;
  infoer.color.a = 1.0;
  infoer.color.r = 1;
  infoer.color.g = 1;
  infoer.color.b = 1;
  infoer.scale.z = 7;
  infoer.text = "ID : " + robot_id_;


  vehicle_model_pub.publish(marker);
  vehicle_info_pub.publish(infoer);

  return true;
}



/** Callbacks **/
void MissionControl::JoyCallback(const sensor_msgs::Joy::ConstPtr& Input) {
  last_timer_ = ros::Time::now();
  static bool isLock = false;
  double axis_deadzone = 0.1;

  (Input->buttons[BUTTON_LB]) ? isJoyControl_ = true : isJoyControl_ = false;

  if (Input->buttons[BUTTON_LB] && Input->buttons[BUTTON_X]) isLock = true;
  if (Input->buttons[BUTTON_LB] && Input->buttons[BUTTON_Y]) isLock = false;

  double axis_x = Input->axes[AXIS_LEFT_UP_DOWN];
  double axis_y = Input->axes[AXIS_RIGHT_LEFT_RIGHT];
  if(fabs(axis_x) < axis_deadzone) axis_x = 0;
  if(fabs(axis_y) < axis_deadzone) axis_y = 0;

  if(isLock) {
    isJoyControl_ = true;
    ZeroCommand(joy_cmd_);
  } else {
    joy_cmd_.linear.x  = axis_x * max_linear_velocity_;
    joy_cmd_.angular.z = axis_y * max_rotation_velocity_;      
  }

  if(isJoyControl_ && Input->buttons[BUTTON_BACK]) {
    global_path_pointcloud_.points.clear();
  }
}

void MissionControl::CmdCallback(const geometry_msgs::Twist::ConstPtr& Input) {
  last_timer_ = ros::Time::now();
  isWIFIControl_ = true;
  geometry_msgs::Twist android_cmd;
  android_cmd.linear.x = Input->linear.x;
  android_cmd.angular.z = Input->angular.z;
  android_cmd.angular.z *= 1.5;
  joy_cmd_ = android_cmd;
}

void MissionControl::TcpCallback(const std_msgs::String::ConstPtr& Input) {

}

int MissionControl::SuperviseDecision(int mission_state){
    int raw_state;
    if(mission_state == static_cast<int>(AutoState::AUTO)){
        MySuperviser_.GenerateTracebackRoute(global_sub_goal_,vehicle_in_map_);   //1
        raw_state = MySuperviser_.AutoSuperviseDecision(obstacle_in_base_,vehicle_in_map_,MyTools_.cmd_vel(),plan_state_,wait_plan_state_); //2
    }
    return raw_state;
}

geometry_msgs::Twist MissionControl::ExecuteDecision(geometry_msgs::Twist Input,int mission_state,int supervise_state){
    geometry_msgs::Twist raw_cmd;
    if(mission_state == static_cast<int>(AutoState::AUTO))
    {
        if(supervise_state == static_cast<int>(AUTO::P)){
            raw_cmd = MyExecuter_.ApplyAutoP(Input);
        }else if(supervise_state == static_cast<int>(AUTO::R1)){
            raw_cmd = MyExecuter_.ApplyAutoR1(Input);
        }else if(supervise_state == static_cast<int>(AUTO::R2)){
            raw_cmd = MyExecuter_.ApplyAutoR2(Input);
        }else if(supervise_state == static_cast<int>(AUTO::R3)){
            raw_cmd = MyExecuter_.ApplyAutoR3(Input);
        }else if(supervise_state == static_cast<int>(AUTO::N)){
            raw_cmd = MyExecuter_.ApplyAutoN(Input);
        }else if(supervise_state == static_cast<int>(AUTO::D1)){
            raw_cmd = MyExecuter_.ApplyAutoD1(Input);
        }else if(supervise_state == static_cast<int>(AUTO::D2)){
            raw_cmd = MyExecuter_.ApplyAutoD2(Input);
        }else if(supervise_state == static_cast<int>(AUTO::D3)){
            raw_cmd = MyExecuter_.ApplyAutoD3(Input);
        }else if(supervise_state == static_cast<int>(AUTO::D4)){
            raw_cmd = MyExecuter_.ApplyAutoD4(Input);
        }else if(supervise_state == static_cast<int>(AUTO::AUTO)){
            raw_cmd = MyExecuter_.ApplyAutoAUTO(Input);
        }else if(supervise_state == static_cast<int>(AUTO::REVOLUTE1)){
            raw_cmd = MyExecuter_.ApplyAutoRevolute1(Input);
            MyExecuter_.ComputeRevoluteCommand(vehicle_in_map_,MySuperviser_.GetRevolutePose(), PI/4, raw_cmd);
        }else if(supervise_state == static_cast<int>(AUTO::REVOLUTE2)){
            raw_cmd = MyExecuter_.ApplyAutoRevolute2(Input);
            MyExecuter_.ComputeRevoluteCommand(vehicle_in_map_,MySuperviser_.GetRevolutePose(), -PI/2, raw_cmd);
        }else if(supervise_state == static_cast<int>(AUTO::TRACEBACK)){
            raw_cmd = TracebackRoute(MySuperviser_.GetVehicleTraceback());
        }
        return raw_cmd;
    }
}

geometry_msgs::Twist MissionControl::TracebackRoute(vector<geometry_msgs::Point32> Input){
    geometry_msgs::Twist raw_cmd;
    static double lookahead_scale = 0.5;
    int lookahead_index = Input.size() * lookahead_scale;
    geometry_msgs::Point32 vehicle_pose_back_local;
    geometry_msgs::Point32 vehicle_pose_backend_local;

    ConvertPoint(Input[lookahead_index], vehicle_pose_back_local, map_to_base_);
    ConvertPoint(Input.front(), vehicle_pose_backend_local, map_to_base_);

    MyController_.ComputePurePursuitCommand(vehicle_pose_backend_local,vehicle_pose_back_local,raw_cmd);
    
    if(CheckTracebackState(Input[lookahead_index])) lookahead_scale = lookahead_scale * 0.5;
    return raw_cmd;
}

bool MissionControl::CheckTracebackState(geometry_msgs::Point32 Input) { 
    double reach_goal_distance = 1;
    geometry_msgs::Point32 Input_local;
    ConvertPoint(Input,Input_local,map_to_base_);
    if(hypot(Input_local.x,Input_local.y) < reach_goal_distance) {
        return true;
    }else{
        return false;
    }
}