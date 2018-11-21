#include "libNaviManager.h"


NaviManager::NaviManager():pn("~"),
isNewGoal(true),isShowBand(false)
{

  plan_sub = n.subscribe("/move_base/GlobalPlanner/plan",1, &NaviManager::plan_callback,this);
  vel_sub = n.subscribe("/base_cmd_vel",1, &NaviManager::vel_callback,this);
  goal_sub = n.subscribe("/move_base_simple/goal",1, &NaviManager::goal_callback,this);

  log_pub = n.advertise<std_msgs::String> ("/ugv_log", 1);
  path_pub = n.advertise<sensor_msgs::PointCloud> ("/path_points", 1);
  waypoint_pub = n.advertise<sensor_msgs::PointCloud> ("/waypoint_points", 1);
  vis_pub = n.advertise<visualization_msgs::Marker>( "/waypoint_marker", 0 );

  sleep(1);

  paramInitialization();
}


NaviManager::~NaviManager(){
}

void NaviManager::Manager()
{
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  recordLog("Navi_manager Node is Running",LogState::INITIALIZATION);
  while(ros::ok())
  {
    NaviManager::Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void NaviManager::paramInitialization()
{
  recordLog("Parameter Initialization Done",LogState::INITIALIZATION);
  robot_position_ = {0,0,0};
}

void NaviManager::Mission()
{
  simDriving(true);
}

void NaviManager::simDriving(bool flag)
{
  if( flag )
  {
    recordLog("Runing Simulation Mode",LogState::STATE_REPORT);

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


void NaviManager::recordLog(string input,LogState level)
{
  std_msgs::String msg;
  string state;
  string event;
  string color;
  int schedule_selected;
  int log_gap_sec;
  static string last_log = "no data";
  static vector<int> log_schedule_ = {0,0,0,0,0};

  switch( level )
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


  if( event != last_log || level != LogState::STATE_REPORT )
  {
    if( log_schedule_[schedule_selected] < int(ros::Time::now().toSec()) - log_gap_sec )
    {
      msg.data = input;
      //ROS_INFO_STREAM(input);

      time_t clock = time(NULL);

      cout<<color<<"Node  | "<<ros::this_node::getName()<<endl;
      cout<<color<<"Data  | "<<asctime(std::localtime(&clock));
      cout<<color<<"State | "<<state<<endl;
      cout<<color<<"Event | "<<event<<endl;
      cout<<ORIGIN<<endl;

      last_log = event;
      log_pub.publish(msg);
      log_schedule_[schedule_selected] = int(ros::Time::now().toSec());
    }
  }

}