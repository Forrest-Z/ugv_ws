#include "libSpeedManager.h"


SpeedManager::SpeedManager():pn("~"),
  joy_speed_(0),joy_rotation_ (0),
  nav_speed_(0),nav_rotation_(0),
  isJoy_(false),isNav_(false),
  isAndroid_(false),isSupJoy_(false),
  isRecovery_(false),
  lost_signal_ct_(0)
{
  pn.param<double>("max_speed", max_speed_, 1.2);
  pn.param<double>("max_rotation", max_rotation_, 0.8);
  pn.param<double>("safe_zone", safe_zone_, 2);
  pn.param<double>("vehicle_radius", vehicle_radius_, 0.5);
  pn.param<double>("radius_scale", radius_scale_, 1.5);
  pn.param<double>("lookahead_time",lookahead_time_,6);
  pn.param<double>("force_constant_x",force_constant_x_,1);
  pn.param<double>("force_constant_y",force_constant_y_,1);
  pn.param<double>("max_height", max_height_, 1);
  pn.param<double>("min_height",min_height_,-0.2);
  pn.param<double>("max_range",max_range_,8);
  pn.param<double>("min_range",min_range_,0.2);
  pn.param<double>("oscillation",oscillation_,0.1);
  pn.param<bool>("one_hand", isOneHand_, false);
  pn.param<vector<double>>("goal_a",goal_a_,{0,0});
  pn.param<vector<double>>("goal_b",goal_b_,{0,0});
  pn.param<string>("base_frame", base_frame_, "/base_link");
  pn.param<string>("camera_frame", camera_frame_, "/power2_point");
  pn.param<string>("lidar_frame", lidar_frame_, "/rslidar");
  pn.param<string>("map_frame", map_frame_, "/map");



  joy_sub = n.subscribe("/joy",1, &SpeedManager::joy_callback,this);
  cmd_sub = n.subscribe("/cmd_vel",1, &SpeedManager::cmd_callback,this);
  local_cmd_sub = n.subscribe("/local_cmd_vel",1, &SpeedManager::local_cmd_callback,this);
  lidar_sub = n.subscribe("/rslidar_points",1, &SpeedManager::lidar_callback,this);
  wall_sub = n.subscribe("/wall_points",1, &SpeedManager::wall_callback,this);
  android_sub = n.subscribe("/virtual_joystick/cmd_vel",1, &SpeedManager::android_callback,this);
  nav_state_sub = n.subscribe("/nav_state",1,&SpeedManager::nav_state_callback,this);
  button_sub = n.subscribe("/button",1,&SpeedManager::button_callback,this);
  power_sub = n.subscribe("/power_points",1,&SpeedManager::power_callback,this);

  vel_pub = n.advertise<geometry_msgs::Twist> ("/base_cmd_vel", 1);
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud> ("/filtered_points", 1);
  visualization_pub = n.advertise<sensor_msgs::PointCloud> ("/visualization_points", 1);
  visualization_pub_2 = n.advertise<sensor_msgs::PointCloud> ("/visualization_points_safe", 1);
  log_pub = n.advertise<std_msgs::String> ("/ugv_log", 1);
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("/navi_goal",1);



  clear_costmap_client = n.serviceClient<std_srvs::EmptyRequest>("/move_base/clear_costmaps");

  sleep(1);

  paramInitialization();
}


SpeedManager::~SpeedManager(){
}

void SpeedManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  recordLog("Speed_manager Node is Running",LogState::INITIALIZATION);
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void SpeedManager::paramInitialization() {
  //recordLog("Parameter Initialization Done",LogState::INITIALIZATION);
}

void SpeedManager::Mission() {
/********************
  CONTROL LOOP
********************/
  if (lost_signal_ct_ > ROS_RATE_HZ * STOP_TIME_SEC) {
    zeroVelocity();
    recordLog("Waiting for Controller",LogState::STATE_REPORT);
  }

  if (isSupJoy_ != true) {
    if (isReceiveCommand()) collisionAvoid();
    else if (isRecovery_) recoveryAcion(); 
  } else {
    superCommand();
  }

  limitCommand(cmd_vel_safe_);
  speedSmoother(cmd_vel_safe_);
  vel_pub.publish(cmd_vel_safe_);
  clearCostmap();

  drawPath(1);
  drawPath(2);

  debugFunction();
}

void SpeedManager::drawVehicle(double center_x,double center_y,int seq) { 
  sensor_msgs::PointCloud pointcloud_vis;
  geometry_msgs::Twist cmd_vel_temp;
  if (seq == 1) cmd_vel_temp = cmd_vel_;
  else if (seq == 2) cmd_vel_temp = cmd_vel_safe_;

  double linear_speed  = cmd_vel_temp.linear.x;
  double angluar_speed = cmd_vel_temp.angular.z;
  double point_step    = 0.01;

  if (linear_speed == 0) {
    for (double i = 0; i < 2 * PI; i+=point_step) {
      geometry_msgs::Point32 point;
      point.x = vehicle_radius_*cos(i);
      point.y = vehicle_radius_*sin(i);
      point.z = 0;
      pointcloud_vis.points.push_back(point);
    }
  } else {
    if (fabs(angluar_speed) < 0.1) {
      //point_step *= 0.5;
      for (double i = 0; i < 2 * lookahead_time_ * (fabs(linear_speed)/max_speed_); i+=point_step) {
        geometry_msgs::Point32 point;

        point.x = sign(linear_speed) * i;
        point.y = radius_scale_ * vehicle_radius_;
        point.z = 0;
        pointcloud_vis.points.push_back(point);

        point.x = sign(linear_speed) * i;
        point.y = -radius_scale_ * vehicle_radius_;
        point.z = 0;
        pointcloud_vis.points.push_back(point);
      }

    } else {
      double moving_radius       = linear_speed/angluar_speed;
      double moving_radius_left  = moving_radius + radius_scale_ * vehicle_radius_;
      double moving_radius_right = moving_radius - radius_scale_ * vehicle_radius_;

      for (double i = 0; i < lookahead_time_ * (fabs(angluar_speed)/max_rotation_); i+=point_step) {
        geometry_msgs::Point32 point;

        point.x = sign(linear_speed) * sign(moving_radius) * moving_radius_left * sin(i);
        point.y = moving_radius - moving_radius_left * cos(i);
        point.z = 0;
        pointcloud_vis.points.push_back(point);

        point.x = sign(linear_speed) * sign(moving_radius) * moving_radius_right * sin(i);
        point.y = moving_radius - moving_radius_right * cos(i);
        point.z = 0;
        pointcloud_vis.points.push_back(point);
      }
    }
  }
  pointcloud_vis.header.frame_id = base_frame_;
  if (seq == 1) visualization_pub.publish(pointcloud_vis);
  else if (seq == 2) visualization_pub_2.publish(pointcloud_vis);
}


void SpeedManager::drawPath(int seq) {
  geometry_msgs::Twist cmd_vel_temp;
  if (seq == 1) cmd_vel_temp = cmd_vel_;
  else if (seq == 2) cmd_vel_temp = cmd_vel_safe_;

  double linear_speed  = cmd_vel_temp.linear.x;
  double angluar_speed = cmd_vel_temp.angular.z;

  double move_step     = 0.3;
  double static sim_timer = 0;
  double center_x,center_y;

  if (linear_speed == 0) {
    center_x  = 0;
    center_y  = 0;
    move_step = 0;
  }
  else if (angluar_speed == 0) {
    center_x = sign(linear_speed) * sim_timer;
    center_y = 0;
    move_step *= 0.5;
  } else {
    double moving_radius       = cmd_vel_temp.linear.x/cmd_vel_temp.angular.z;
    center_x = moving_radius * sin( cmd_vel_temp.angular.z*sim_timer );
    center_y = moving_radius - moving_radius * cos( cmd_vel_temp.angular.z*sim_timer );
  }

  drawVehicle(center_x,center_y,seq);
  (sim_timer < lookahead_time_) ? sim_timer+=move_step : sim_timer = 0 ;
}

bool SpeedManager::collisionCheck(double input_x, double input_y) {
  //limitCommand(cmd_vel_);
  double linear_speed  = cmd_vel_.linear.x;
  double angluar_speed = cmd_vel_.angular.z;
  double lookahead_index = 2;

  if (linear_speed == 0 && angluar_speed == 0) return false;

  if (linear_speed == 0 && angluar_speed != 0) {
    double distance = hypot(input_x,input_y);
    return distance <  vehicle_radius_;
  }

  if (linear_speed != 0 && angluar_speed == 0) {
    if (fabs(input_y) > radius_scale_ * vehicle_radius_) return false;
    else {
      if (linear_speed > 0) 
        return (input_x < lookahead_index * lookahead_time_ * linear_speed) && (input_x > 0);
      else 
        return (fabs(input_x) < fabs(lookahead_index * lookahead_time_ 
          * linear_speed)) && (input_x < 0);
    }
  }


  if (linear_speed != 0 && angluar_speed != 0) {
    double distance = hypot(input_x,input_y);
    if (distance <  2 * radius_scale_ * vehicle_radius_ && 
      sign(linear_speed) == sign(input_x)) return true;

    double moving_radius       = linear_speed/angluar_speed;
    double moving_radius_left  = moving_radius + radius_scale_ * vehicle_radius_;
    double moving_radius_right = moving_radius - radius_scale_ * vehicle_radius_;

    double unit_dis_left  = asin( input_x/moving_radius_left );
    double unit_dis_right = asin( input_x/moving_radius_right );

//    double distance = hypot(input_x,input_y);
    if (distance > lookahead_time_ * lookahead_index * fabs(linear_speed)) return false;

    double safe_y_left   = moving_radius - 
      moving_radius_left * cos(sign(angluar_speed)*unit_dis_left);
    double safe_y_right  = moving_radius - 
      moving_radius_right * cos(sign(angluar_speed)*unit_dis_right);

    return input_y < safe_y_right && input_y > safe_y_left && sign(input_x) == sign(linear_speed);
  }

}

void SpeedManager::computeForce(sensor_msgs::PointCloud& points,
  double& force_x,double& force_y,double& freespace,bool& isEmergency) {

  double distance;
  double coordinate_x;
  double coordinate_y;
  double certainty = 0.01;

  if (!points.points.empty()) { 
    for (int i = 0; i < points.points.size(); ++i) {
      distance = hypot(points.points[i].x,points.points[i].y);
      coordinate_x = points.points[i].x;
      coordinate_y = points.points[i].y;

      if (distance > safe_zone_) continue;
      if (distance < vehicle_radius_) isEmergency = true;

      if (coordinate_y > radius_scale_ * vehicle_radius_) coordinate_y = 0;

      force_x += (certainty*force_constant_x_*coordinate_x)/pow(distance,3);
      force_y += (certainty*force_constant_y_*coordinate_y)/pow(distance,2);

      if (coordinate_y >= 0) freespace++;
      else freespace--;
    }
  }

}


void SpeedManager::collisionAvoid()
{
  // Virtual force field
  // F(i,j) = [Fcr * C(i,j)/d(i,j)^2]  *  [xt-xo/d(i,j) + yt-yo/d(i,j)]

  cmd_vel_safe_ = cmd_vel_;

  double free_space_index = 0;
  double virtual_force_x  = 0;
  double virtual_force_y  = 0;

  bool isEmergency = false;

  // pointcloud_lidar.header.frame_id = base_frame_;
  collision_wall_points_.header.frame_id = base_frame_;
  //collision_lidar_points_.header.frame_id = base_frame_;
  pointcloud_pub.publish(collision_wall_points_);

  computeForce(collision_lidar_points_,virtual_force_x,virtual_force_y,free_space_index,isEmergency);
  computeForce(collision_wall_points_,virtual_force_x,virtual_force_y,free_space_index,isEmergency);
  computeForce(collision_power_points_,virtual_force_x,virtual_force_y,free_space_index,isEmergency);
  //ROS_INFO_STREAM("FOC = "<<virtual_force_x<<","<<virtual_force_y);

  isFreeDrive_ = (collision_wall_points_.points.empty() && collision_lidar_points_.points.empty());


  // if (fabs(cmd_vel_.linear.x) < fabs(virtual_force_x)) cmd_vel_safe_.linear.x = 0;
  // else cmd_vel_safe_.linear.x -= virtual_force_x;
  cmd_vel_safe_.linear.x -= virtual_force_x;

  if (cmd_vel_.linear.x >= 0) cmd_vel_safe_.angular.z -= virtual_force_y;
  else cmd_vel_safe_.angular.z += virtual_force_y;

  //ROS_INFO_STREAM("SPE = "<<cmd_vel_safe_.linear.x<<","<<cmd_vel_safe_.angular.z);

  if (virtual_force_x != 0) {
    cmd_vel_safe_.angular.z -= free_space_index * 0.01;
  }

  if (isEmergency) {
    recordLog("Too Close to Obstacle",LogState::ERROR);
    cmd_vel_safe_.linear.x = 0;
    cmd_vel_safe_.angular.z = cmd_vel_.angular.z * 0.3;
  }

}


void SpeedManager::limitCommand(geometry_msgs::Twist& input_cmd)
{
  if (fabs(input_cmd.linear.x) > fabs(max_speed_)) input_cmd.linear.x 
    = sign(input_cmd.linear.x) * max_speed_;
  if (fabs(input_cmd.angular.z) > fabs(max_rotation_)) input_cmd.angular.z 
    = sign(input_cmd.angular.z) * max_rotation_;
  if (fabs(input_cmd.angular.z) < oscillation_) input_cmd.angular.z = 0;
  // if (fabs(input_cmd.linear.x) < oscillation_) input_cmd.linear.x = 0;
}

void SpeedManager::speedSmoother(geometry_msgs::Twist& input_cmd)
{
  static double last_cmd_linear = 0;
  static double last_cmd_anglar = 0;

  double max_acc_rt = 5;
  double max_acc_loop = max_acc_rt/ROS_RATE_HZ;

  if (input_cmd.linear.x > last_cmd_linear) {
    input_cmd.linear.x = last_cmd_linear + max_acc_loop;
    last_cmd_linear = input_cmd.linear.x;
  }
  if (input_cmd.angular.z > last_cmd_anglar && input_cmd.angular.z >= 0) {
    input_cmd.angular.z = last_cmd_anglar + max_acc_loop;
    last_cmd_anglar = input_cmd.angular.z;
  }
  else if (input_cmd.angular.z < last_cmd_anglar && input_cmd.angular.z < 0) {
    input_cmd.angular.z = last_cmd_anglar - max_acc_loop;
    last_cmd_anglar = input_cmd.angular.z;
  }

}


bool SpeedManager::isReceiveCommand()
{ 
  static double last_cmd_linear = -99;
  static double last_cmd_anglar = -99;
  static int stuck_ct = 0;
  int stuck_limit = 5;

  if (isJoy_) {
    cmd_vel_.linear.x  = joy_speed_;
    cmd_vel_.angular.z = joy_rotation_;
    lost_signal_ct_ = 0;
    recordLog("Using JOY",LogState::STATE_REPORT);
    return true;    
  }

  if (isAndroid_) {
    cmd_vel_.linear.x  = android_speed_;
    cmd_vel_.angular.z = android_rotation_;
    lost_signal_ct_ = 0;
    recordLog("Using ANDROID",LogState::STATE_REPORT);
    return true;
  }
     
  if (last_cmd_linear == nav_speed_ && last_cmd_anglar == nav_rotation_) {
    if (stuck_ct > stuck_limit) isNav_ = false;
    else stuck_ct++;
  } else {
    last_cmd_linear = nav_speed_;
    last_cmd_anglar = nav_rotation_;
    stuck_ct = 0;
  }

  if (isNav_) {
    cmd_vel_.linear.x  = nav_speed_;
    cmd_vel_.angular.z = nav_rotation_;
    lost_signal_ct_ = 0;
    recordLog("Using NAV",LogState::STATE_REPORT);
    return true;
  } 

  zeroVelocity();
  lost_signal_ct_++;
  return false;
}

void SpeedManager::recordLog(string input,LogState level) {
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


void SpeedManager::clearCostmap() {
  static int last_clear_time = 0;
  std_srvs::Empty reset;
  if (last_clear_time <  int(ros::Time::now().toSec())) {
    clear_costmap_client.call(reset);
    last_clear_time = int(ros::Time::now().toSec());
  }
}

void SpeedManager::superCommand() {
  recordLog("SUPER DRIVER!",LogState::WARNNING);
  cmd_vel_safe_.linear.x = joy_speed_;
  cmd_vel_safe_.angular.z = joy_rotation_;
  cmd_vel_.linear.x = joy_speed_;
  cmd_vel_.angular.z = joy_rotation_;
  lost_signal_ct_ = 0;
}

void SpeedManager::recoveryAcion() {
  recordLog("Trying Recovery Action",LogState::STATE_REPORT);
  lost_signal_ct_ = 0;
}


void SpeedManager::debugFunction() {
  //recordLog("Debug Test",LogState::WARNNING);
}