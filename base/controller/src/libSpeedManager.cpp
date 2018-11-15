#include "libSpeedManager.h"


SpeedManager::SpeedManager():pn("~"),
  joy_speed_(0),joy_rotation_ (0),
  nav_speed_(0),nav_rotation_(0),
  isJoy_(false),isNav_(false),
  isAndroid_(false),isSupJoy_(false),
  lost_signal_ct_(0),last_log_("empty log")
{
  pn.param<double>("max_speed", max_speed_, 1.2);
  pn.param<double>("max_rotation", max_rotation_, 0.8);
  pn.param<double>("safe_zone", safe_zone_, 2);
  pn.param<double>("vehicle_radius", vehicle_radius_, 0.5);
  pn.param<double>("lookahead_time",lookahead_time_,6);
  pn.param<double>("force_constant_x",force_constant_x_,1);
  pn.param<double>("force_constant_y",force_constant_y_,1);

  joy_sub = n.subscribe("/joy",1, &SpeedManager::joy_callback,this);
  cmd_sub = n.subscribe("/cmd_vel",1, &SpeedManager::cmd_callback,this);
  scan_sub = n.subscribe("/scan",1, &SpeedManager::scan_callback,this);
  lidar_sub = n.subscribe("/rslidar_points",1, &SpeedManager::lidar_callback,this);
  android_sub = n.subscribe("/virtual_joystick/cmd_vel",1, &SpeedManager::android_callback,this);
  nav_state_sub = n.subscribe("/nav_state",1,&SpeedManager::nav_state_callback,this);
  button_sub = n.subscribe("/button",1,&SpeedManager::button_callback,this);

  vel_pub = n.advertise<geometry_msgs::Twist> ("/base_cmd_vel", 1);
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud> ("/filtered_points", 1);
  visualization_pub = n.advertise<sensor_msgs::PointCloud> ("/visualization_points", 1);
  visualization_pub_2 = n.advertise<sensor_msgs::PointCloud> ("/visualization_points_safe", 1);
  log_pub = n.advertise<std_msgs::String> ("/ugv_log", 1);
  move_base_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
  move_base_cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);


  sleep(1);

  paramInitialization();
}


SpeedManager::~SpeedManager(){
}

void SpeedManager::Manager()
{
  ros::Rate loop_rate(ROS_RATE_HZ);
  recordLog("Speed_manager Node is Running",LogState::INITIALIZATION);

  while(ros::ok())
  {
    SpeedManager::Mission();
    loop_rate.sleep();
    ros::spinOnce();    
  }
}

void SpeedManager::paramInitialization()
{
  recordLog("Parameter Initialization Done",LogState::INITIALIZATION);
}

void SpeedManager::Mission()
{
/********************
  CONTROL LOOP
********************/
  if (lost_signal_ct_ > ROS_RATE_HZ * STOP_TIME_SEC )
  {
    zeroVelocity();
    recordLog("Waiting for Controller",LogState::STATE_REPORT);
  }

  if(isSupJoy_ != true)
  {
    if( isReceiveCommand(ControlState::JOY_CONTROL) ) collisionAvoid();
  } else {
    cmd_vel_safe_.linear.x = joy_speed_;
    cmd_vel_safe_.angular.z = joy_rotation_;
    lost_signal_ct_ = 0;
    recordLog("SUPER DRIVER!",LogState::WARNNING);
  }

  limitCommand();
  vel_pub.publish(cmd_vel_safe_);

  drawPath(1);
  drawPath(2);

  debugFunction();

}

void SpeedManager::drawVehicle(double center_x,double center_y,int seq)
{ 
  sensor_msgs::PointCloud pointcloud_vis;
  geometry_msgs::Twist cmd_vel_temp;
  if(seq == 1) cmd_vel_temp = cmd_vel_;
  else if(seq == 2) cmd_vel_temp = cmd_vel_safe_;

  double linear_speed  = cmd_vel_temp.linear.x;
  double angluar_speed = cmd_vel_temp.angular.z;
  double point_step    = 0.01;

  if(linear_speed == 0)
  {
    for(double i = 0; i < 2 * PI; i+=point_step)
    {
      geometry_msgs::Point32 point;
      point.x = vehicle_radius_*cos(i);
      point.y = vehicle_radius_*sin(i);
      point.z = 0;
      pointcloud_vis.points.push_back(point);
    }
  } else {

    if( fabs(angluar_speed) < 0.1 )
    {
      //point_step *= 0.5;
      for( double i = 0; i < 2 * lookahead_time_; i+=point_step )  //i+=(point_step/fabs(linear_speed+0.01))
      {
        geometry_msgs::Point32 point;

        point.x = sign(linear_speed) * i;
        point.y = vehicle_radius_;
        point.z = 0;
        pointcloud_vis.points.push_back(point);

        point.x = sign(linear_speed) * i;
        point.y = -vehicle_radius_;
        point.z = 0;
        pointcloud_vis.points.push_back(point);
      }

    } else {
      double moving_radius       = linear_speed/angluar_speed;
      double moving_radius_left  = moving_radius + vehicle_radius_;
      double moving_radius_right = moving_radius - vehicle_radius_;

      for( double i = 0; i < lookahead_time_; i+=point_step )
      {
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
  pointcloud_vis.header.frame_id = "base_link";
  if(seq == 1) visualization_pub.publish(pointcloud_vis);
  else if(seq == 2) visualization_pub_2.publish(pointcloud_vis);
}


void SpeedManager::drawPath(int seq)
{
  geometry_msgs::Twist cmd_vel_temp;
  if(seq == 1) cmd_vel_temp = cmd_vel_;
  else if(seq == 2) cmd_vel_temp = cmd_vel_safe_;

  double linear_speed  = cmd_vel_temp.linear.x;
  double angluar_speed = cmd_vel_temp.angular.z;

  double move_step     = 0.3;
  double static sim_timer = 0;
  double center_x,center_y;

  if( linear_speed == 0 )
  {
    center_x  = 0;
    center_y  = 0;
    move_step = 0;
  }
  else if( angluar_speed == 0 )
  {
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

bool SpeedManager::collisionCheck(double input_x, double input_y)
{
  double linear_speed  = cmd_vel_.linear.x;
  double angluar_speed = cmd_vel_.angular.z;
  double lookahead_index = 2;

  if( linear_speed == 0 && angluar_speed == 0 ) return false;

  if( linear_speed == 0 && angluar_speed != 0 )
  {
    double distance = hypot(input_x,input_y);
    return (distance < vehicle_radius_);
  }

  if( linear_speed != 0 && angluar_speed == 0 )
  {
    if( fabs(input_y) > vehicle_radius_ ) return false;
    else
    {
      if( linear_speed > 0 ) 
        return ( (input_x < lookahead_index * lookahead_time_ * linear_speed) && (input_x > 0) );
      else 
        return ( (fabs(input_x) < fabs(lookahead_index * lookahead_time_ * linear_speed)) && (input_x < 0) );
    }
  }


  if( linear_speed != 0 && angluar_speed != 0 )
  {
    bool isleft  = false;
    bool isright = false;

    double moving_radius       = linear_speed/angluar_speed;
    double moving_radius_left  = moving_radius + vehicle_radius_;
    double moving_radius_right = moving_radius - vehicle_radius_;

    double unit_dis_left  = asin( input_x/moving_radius_left );
    double unit_dis_right = asin( input_x/moving_radius_right );

    double distance = hypot(input_x,input_y);
    if( distance > lookahead_time_ * lookahead_index * fabs(linear_speed) ) return false;

    double safe_y_left   = moving_radius - moving_radius_left * cos(sign(angluar_speed)*unit_dis_left);
    double safe_y_right  = moving_radius - moving_radius_right * cos(sign(angluar_speed)*unit_dis_right);

    return ( input_y < safe_y_right && input_y > safe_y_left && sign(input_x) == sign(linear_speed) );
  }

}

void SpeedManager::collisionAvoid()
{
  // Virtual force field
  // F(i,j) = [Fcr * C(i,j)/d(i,j)^2]  *  [xt-xo/d(i,j) + yt-yo/d(i,j)]

  cmd_vel_safe_ = cmd_vel_;

  double virtual_force_x = 0;
  double virtual_force_y = 0;

  double certainty = 0.01;

  bool static isEmergency = false;

  if( !collision_points_.points.empty() )
  { 
    isEmergency = false;

    for (int i = 0; i < collision_points_.points.size(); ++i)
    {
      double distance = hypot(collision_points_.points[i].x,collision_points_.points[i].y);
      double coordinate_x = collision_points_.points[i].x;
      double coordinate_y = collision_points_.points[i].y;

      if ( distance > safe_zone_ ) continue;

      if ( distance < vehicle_radius_ ) isEmergency = true;

      virtual_force_x += (certainty*force_constant_x_*coordinate_x)/pow(distance,3);
      virtual_force_y += (certainty*force_constant_y_*coordinate_y)/pow(distance,3);
    }
    //ROS_INFO_STREAM("FOC = "<<virtual_force_x<<","<<virtual_force_y);
    if(fabs(cmd_vel_.linear.x) < fabs(virtual_force_x)) cmd_vel_safe_.linear.x = 0;
    else cmd_vel_safe_.linear.x -= virtual_force_x;

    if(cmd_vel_.linear.x >= 0) cmd_vel_safe_.angular.z -= virtual_force_y;
    else cmd_vel_safe_.angular.z += virtual_force_y;
    //ROS_INFO_STREAM("SPE = "<<cmd_vel_safe_.linear.x<<","<<cmd_vel_safe_.angular.z);
  }

  if(isEmergency) 
  {
    recordLog("Too Close to Obstacle",LogState::ERROR);
    cmd_vel_safe_.linear.x = 0;
    cmd_vel_safe_.angular.z = 0;
  }

}


void SpeedManager::limitCommand()
{
  if( fabs(cmd_vel_.linear.x) > fabs(max_speed_) ) cmd_vel_.linear.x 
    = sign(cmd_vel_.linear.x) * max_speed_;

  if( fabs(cmd_vel_.angular.z) > fabs(max_rotation_) ) cmd_vel_.angular.z 
    = sign(cmd_vel_.angular.z) * max_rotation_;
}

bool SpeedManager::isReceiveCommand(const ControlState input)
{ 
  switch(input)
  {
    case ControlState::JOY_CONTROL:
    {
      if( isJoy_ )
      {
        cmd_vel_.linear.x  = joy_speed_;
        cmd_vel_.angular.z = joy_rotation_;
        lost_signal_ct_ = 0;
        recordLog("Using JOY",LogState::STATE_REPORT);
        return true;    
      } 
    }
    //break;

    case ControlState::ANDROID_CONTROL:
    {     
      if( isAndroid_ )
      {
        cmd_vel_.linear.x  = android_speed_;
        cmd_vel_.angular.z = android_rotation_;
        lost_signal_ct_ = 0;
        recordLog("Using ANDROID",LogState::STATE_REPORT);
        return true;
      } 
    }
    //break;

    case ControlState::NAV_CONTROL:
    {      
      if( isNav_ )
      {
        cmd_vel_.linear.x  = nav_speed_;
        cmd_vel_.angular.z = nav_rotation_;
        lost_signal_ct_ = 0;
        recordLog("Using NAV",LogState::STATE_REPORT);
        return true;
      } 
    }
    break;    

    default:
      break;
  }
  zeroVelocity();
  lost_signal_ct_++;
  return false;
}

void SpeedManager::recordLog(string input,LogState level)
{
  std_msgs::String msg;
  string state;
  string event;
  string color;
  int schedule_selected;
  int log_gap_sec;

  switch(level)
  {
    case LogState::INITIALIZATION:
    {
      state = "Initialization";
      event = input;
      schedule_selected = 0;
      log_gap_sec = -1;
      color = WHITE;
    }
    break;
    
    case LogState::INFOMATION:
    {
      state = "Infomation";
      event = input;
      schedule_selected = 0;
      log_gap_sec = -1;
      color = WHITE;
    }
    break;

    case LogState::STATE_REPORT:
    {
      state = "Controller State";
      event = input;
      schedule_selected = 1;
      log_gap_sec = 1;
      color = GREEN;
    }
    break;

    case LogState::WARNNING:
    {
      state = "WARNNING";
      event = input;
      schedule_selected = 2;
      log_gap_sec = 1;
      color = YELLOW;
    }
    break;

    case LogState::ERROR:
    {
      state = "ERROR";
      event = input;
      schedule_selected = 3;
      log_gap_sec = -1;
      color = RED;
    }
    break;

    default:
      break;
  }


  if(event != last_log_ || level != LogState::STATE_REPORT)
  {
    if( log_schedule_[schedule_selected] < int(ros::Time::now().toSec()) - log_gap_sec )
    {
      msg.data = input;
      //ROS_INFO_STREAM(input);

      time_t clock = time(NULL);

      std::cout<<color<<"Node  | "<<ros::this_node::getName()<<endl;
      std::cout<<color<<"Data  | "<<asctime(std::localtime(&clock));
      std::cout<<color<<"State | "<<state<<endl;
      std::cout<<color<<"Event | "<<event<<endl;
      std::cout<<ORIGIN<<endl;

      last_log_ = event;
      log_pub.publish(msg);
      log_schedule_[schedule_selected] = int(ros::Time::now().toSec());
    }
  }

}

void SpeedManager::debugFunction()
{
  //recordLog("Debug Test",LogState::WARNNING);
}