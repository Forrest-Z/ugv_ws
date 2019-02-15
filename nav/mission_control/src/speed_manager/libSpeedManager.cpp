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
  pn.param<vector<double>>("goal_x",goal_x_,{0,0});
  pn.param<vector<double>>("goal_y",goal_y_,{0,0});
  pn.param<string>("base_frame", base_frame_, "/base_link");
  pn.param<string>("camera_frame", camera_frame_, "/power2_point");
  pn.param<string>("lidar_frame", lidar_frame_, "/rslidar");
  pn.param<string>("map_frame", map_frame_, "/map");
  pn.param<double>("play_speed",play_speed_,1);



  joy_sub = n.subscribe("/joy",1, &SpeedManager::joy_callback,this);
  cmd_sub = n.subscribe("/cmd_vel",1, &SpeedManager::cmd_callback,this);
  local_cmd_sub = n.subscribe("/local_cmd_vel",1, &SpeedManager::local_cmd_callback,this);
  lidar_sub = n.subscribe("/rslidar_points",1, &SpeedManager::lidar_callback,this);
  wall_sub = n.subscribe("/wall_points",1, &SpeedManager::wall_callback,this);
  android_sub = n.subscribe("/virtual_joystick/cmd_vel",1, &SpeedManager::android_callback,this);
  nav_state_sub = n.subscribe("/navi_state",1,&SpeedManager::nav_state_callback,this);
  button_sub = n.subscribe("/button",1,&SpeedManager::button_callback,this);
  power_sub = n.subscribe("/power_points",1,&SpeedManager::power_callback,this);

  vel_pub = n.advertise<geometry_msgs::Twist> ("/base_cmd_vel", 1);
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud> ("/filtered_points", 1);
  visualization_pub = n.advertise<sensor_msgs::PointCloud> ("/visualization_points", 1);
  visualization_pub_2 = n.advertise<sensor_msgs::PointCloud> ("/visualization_points_safe", 1);
  log_pub = n.advertise<std_msgs::String> ("/ugv_log", 1);
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("/navi_goal",1);
  lane_pub = n.advertise<sensor_msgs::PointCloud> ("/lane_points", 1);


  clear_costmap_client = n.serviceClient<std_srvs::EmptyRequest>("/move_base/clear_costmaps");

  sleep(1);

  paramInitialization();
}


SpeedManager::~SpeedManager(){
}

void SpeedManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ * play_speed_);
  recordLog("Speed_manager Node is Running",LogState::INITIALIZATION);
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void SpeedManager::paramInitialization() {
  //recordLog("Parameter Initialization Done",LogState::INITIALIZATION);
  // cv::namedWindow("Map", CV_GUI_EXPANDED);
  // cv::moveWindow("Map",400,300);
  

}

void SpeedManager::Mission() {
  if (lost_signal_ct_ > ROS_RATE_HZ * STOP_TIME_SEC) {
    zeroVelocity();
    //recordLog("Waiting for Controller",LogState::STATE_REPORT);
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

  double distance = hypot(input_x,input_y);

  if (linear_speed == 0 && angluar_speed == 0) return false;

  if (linear_speed == 0 && angluar_speed != 0) {
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
    if (distance <  2 * radius_scale_ * vehicle_radius_ && 
      sign(linear_speed) == sign(input_x)) return true;
    if (distance > lookahead_time_ * lookahead_index * fabs(linear_speed)) return false;

    double moving_radius = fabs(linear_speed/angluar_speed);
    double limit_factor  = 0.2;

    double safe_x_front = radius_scale_ * vehicle_radius_ 
      + linear_speed/max_speed_ * moving_radius;
    double safe_x_rear = - radius_scale_ * vehicle_radius_ 
      - linear_speed/max_speed_ * moving_radius;
    bool output_result_x = (linear_speed > 0) ? 
      (input_x > -radius_scale_ * vehicle_radius_ && input_x < safe_x_front) : 
      (input_x > safe_x_rear && input_x < radius_scale_ * vehicle_radius_);

    double safe_y_left  = radius_scale_ * vehicle_radius_ 
      + limit_factor * linear_speed/max_speed_ * moving_radius * sign(linear_speed);
    double safe_y_right = - radius_scale_ * vehicle_radius_ 
      - limit_factor * linear_speed/max_speed_ * moving_radius * sign(linear_speed);
    bool output_result_y = (sign(angluar_speed*linear_speed) > 0) ? 
      (input_y < safe_y_left && input_y > - radius_scale_ * vehicle_radius_) : 
      (input_y < radius_scale_ * vehicle_radius_ && input_y > safe_y_right);

    // cout << "Range checK"
    //   << " y left  = " << safe_y_left
    //   << " y right = " << safe_y_right
    //   << " x front = " << safe_x_front
    //   << " x rear  = " << safe_x_rear << endl;

    return output_result_x && output_result_y;
    
    // double moving_radius       = linear_speed/angluar_speed;
    // double moving_radius_left  = moving_radius + radius_scale_ * vehicle_radius_;
    // double moving_radius_right = moving_radius - radius_scale_ * vehicle_radius_;

    // double unit_dis_left  = asin( input_x/moving_radius_left );
    // double unit_dis_right = asin( input_x/moving_radius_right );

    // double safe_y_left   = moving_radius - 
    //   moving_radius_left * cos(sign(angluar_speed)*unit_dis_left);
    // double safe_y_right  = moving_radius - 
    //   moving_radius_right * cos(sign(angluar_speed)*unit_dis_right);

    // return input_y < safe_y_right && input_y > safe_y_left && sign(input_x) == sign(linear_speed);
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


bool SpeedManager::generateLane(sensor_msgs::PointCloud Input,LaneFrame& Output) {
  
  if(Input.points.size() == 0) return false;

  sensor_msgs::PointCloud2 pointcloud_2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl_1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl_2 (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::convertPointCloudToPointCloud2(Input,pointcloud_2);

  pcl::fromROSMsg(pointcloud_2,*pointcloud_pcl_1);

  pcl::ModelCoefficients::Ptr coefficients_1 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);

  seg.setModelType (pcl::SACMODEL_LINE); //SACMODEL_PARALLEL_LINE
  seg.setMethodType (pcl::SAC_RANSAC);

  seg.setDistanceThreshold (0.2);
  seg.setInputCloud (pointcloud_pcl_1);
  seg.segment (*inliers_1,*coefficients_1);
  if (inliers_1->indices.size () == 0) return false;

  double kx = coefficients_1->values[3];
  double ky = coefficients_1->values[4];

  double line_1_x = coefficients_1->values[0];
  double line_1_y = coefficients_1->values[1];


  int buffer_size = 10;
  static int buffer_ct = 0;
  double shift_threshold = 0.3;

  static double last_kx = kx;
  static double last_ky = ky;

  if(fabs(last_kx - kx) > shift_threshold || fabs(last_ky - ky) > shift_threshold) {
    buffer_ct = 0;
    last_kx = kx;
    last_ky = ky;
  } else {
    buffer_ct++;
  }
  if(buffer_ct < buffer_size) return false;



  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (pointcloud_pcl_1);
  extract.setIndices (inliers_1);
  extract.setNegative (true);
  extract.filter (*pointcloud_pcl_2);

  seg.setInputCloud (pointcloud_pcl_2);
  seg.segment (*inliers_2,*coefficients_2);
  if (inliers_2->indices.size () == 0) return false;

  double line_2_x = coefficients_2->values[0];
  double line_2_y = coefficients_2->values[1];

  static double last_dis_1;
  static double last_dis_2;
  static double last_dis;
  static bool isInit = false;
  if(!isInit) {
    last_dis = hypot((line_1_x - line_2_x),(line_1_y - line_2_y));
    if(line_1_y > 0) {
      last_dis_1 = hypot((line_1_x),(line_1_y));
      last_dis_2 = hypot((line_2_x),(line_2_y));
    } else {
      last_dis_1 = hypot((line_2_x),(line_2_y));
      last_dis_2 = hypot((line_1_x),(line_1_y));
    }
    isInit = true;
  }

  double dis_total = hypot((line_1_x - line_2_x),(line_1_y - line_2_y));
  double dis_1,dis_2;
  if(line_1_y > 0) {
    dis_1 = hypot((line_1_x),(line_1_y));
    dis_2 = hypot((line_2_x),(line_2_y));
  } else {
    dis_1 = hypot((line_2_x),(line_2_y));
    dis_2 = hypot((line_1_x),(line_1_y));
  }

  // if(fabs(last_dis-dis_total)>shift_threshold) {
  //   last_dis = dis_total;
  //   return false;
  // } else {
  //   last_dis = dis_total;
  // }
  // if(fabs(last_dis_1-dis_1)>shift_threshold) {
  //   dis_1 = last_dis_1;
  //   return false;
  // } else {
  //   last_dis_1 = dis_1;
  // }
  // if(fabs(last_dis_2-dis_2)>shift_threshold) {
  //   dis_2 = last_dis_2;
  //   return false;
  // } else {
  //   last_dis_2 = dis_2;
  // }




  // cout << "Coefficients  " << endl
  //   << "kx = " << kx << "  ky = "<< ky<< endl
  //   << "kx/ky = " << kx/ky << " ky/kx = "<<ky/kx <<endl
  //   << "tan-1 (kx/ky) = " << atan2(kx,ky)*180/PI << endl
  //   << "tan-1 (ky/kx) = " << atan2(ky,kx)*180/PI << endl
  // << endl;
  //   << "Road width " <<dis_total
  //   << "  Dis left " << dis_1
  //   << "  Dis right " << dis_2 <<endl;


  Output.distance.left = dis_1;
  Output.distance.right = dis_2;
  Output.direction = -atan2(ky,kx);

  sensor_msgs::PointCloud pointcloud_lane;
  pointcloud_lane.header.frame_id = base_frame_;
  for (int i = 0; i < inliers_1->indices.size(); i++) {
    geometry_msgs::Point32 point;
    point.x = pointcloud_pcl_1->points[inliers_1->indices[i]].x;
    point.y = pointcloud_pcl_1->points[inliers_1->indices[i]].y;
    pointcloud_lane.points.push_back(point);
  }

  for (int i = 0; i < inliers_2->indices.size(); i++) {
    geometry_msgs::Point32 point;
    point.x = pointcloud_pcl_2->points[inliers_2->indices[i]].x;
    point.y = pointcloud_pcl_2->points[inliers_2->indices[i]].y;
    pointcloud_lane.points.push_back(point);
  }
  lane_pub.publish(pointcloud_lane);

  return true;
}

void SpeedManager::collisionAvoid()
{
  // Virtual force field
  // F(i,j) = [Fcr * C(i,j)/d(i,j)^2]  *  [xt-xo/d(i,j) + yt-yo/d(i,j)]
  limitCommand(cmd_vel_);
  cmd_vel_safe_ = cmd_vel_;

  double free_space_index = 0;
  double virtual_force_x  = 0;
  double virtual_force_y  = 0;

  bool isEmergency = false;

  vector<sensor_msgs::PointCloud> total_collision_points;
  total_collision_points =
    {collision_lidar_points_,collision_wall_points_all_,collision_power_points_};

  vector<sensor_msgs::PointCloud> part_collision_points;
  part_collision_points =
    {collision_lidar_points_,collision_wall_points_,collision_power_points_};

  filterPointCloud(total_collision_points);

  Mat map_grid;


  // cout << "num " << obs_points_number <<endl;
  // cout<< "cmd vel = " << cmd_vel_.linear.x << endl;
  // cout<< "safe vel = " << cmd_vel_safe_.linear.x << endl;

  static int stop_timer = 0;
  static bool isStopped = false;
  int obs_points_number = makeDecision(part_collision_points);

  if(!isStopped) {
    if(stop_timer > ROS_RATE_HZ * STOP_TIME_SEC) isStopped = true;
    if(obs_points_number >= 0  && fabs(cmd_vel_safe_.angular.z) < oscillation_) {
      cmd_vel_safe_.linear.x -= stop_timer * (max_speed_/(ROS_RATE_HZ * STOP_TIME_SEC));
      if(cmd_vel_safe_.linear.x < 0) cmd_vel_safe_.linear.x = 0;
      stop_timer++;
      return;
    }
  }
  if(obs_points_number == -1) {
    isStopped = false;
    stop_timer = 0;
  }



  //int current_state = findVehicleState(total_collision_points);

  // publishPathParticle(total_collision_points,map_grid);
  // findClearPath(map_grid,cmd_vel_safe_);

  // if(findVehicleState(total_collision_points)!=3) {
  //   findClearPath(map_grid,cmd_vel_safe_);
  // }

  // LaneFrame lane_pos;


  // if(generateLane(collision_wall_points_all_,lane_pos)) {
  //   // cout << "Vehicle Angle = " << lane_pos.direction << endl
  //   //   << " Vehicle to Right Edge = " << lane_pos.distance.right << endl
  //   //   << " Vehicle to Left  Edge = " << lane_pos.distance.left << endl;
  //   findAvailableLane(lane_pos,total_collision_points);
  // }


  computeForce(collision_lidar_points_,virtual_force_x,virtual_force_y,free_space_index,isEmergency);
  computeForce(collision_wall_points_,virtual_force_x,virtual_force_y,free_space_index,isEmergency);
  computeForce(collision_power_points_,virtual_force_x,virtual_force_y,free_space_index,isEmergency);
  //ROS_INFO_STREAM("FOC = "<<virtual_force_x<<","<<virtual_force_y);


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

void SpeedManager::findAvailableLane(LaneFrame Input,std::vector<sensor_msgs::PointCloud> Input_Points) {
  int buffer_scale = 3;
  double lane_width = Input.distance.right + Input.distance.left;
  int allow_lane_num = lane_width / (vehicle_radius_ * buffer_scale);
  int vehicle_current_lane_num = Input.distance.right / (vehicle_radius_ * buffer_scale);

  cout << "There are " << allow_lane_num << " Lane "<< endl
    << "Vehicle at Lane " << vehicle_current_lane_num << endl;


  int move_to_lane = findObstacleLane(Input,Input_Points,allow_lane_num,vehicle_current_lane_num,buffer_scale);
  
}

int SpeedManager::findObstacleLane(LaneFrame Input, std::vector<sensor_msgs::PointCloud> Input_Points,int Lane_Num, int Vehicle_Lane, int Scale) {

  vector<int> invailed_lane(Lane_Num,0);
  cout<<"check " <<invailed_lane.size()<<endl;

  double max_dis = 0;
  double min_dis = DBL_MAX;

  for (int m = 0; m < Input_Points.size(); m++) {
    if(Input_Points[m].points.empty()) continue;
    for (int n = 0; n < Input_Points[m].points.size(); n++) {

      double lane_y = Input_Points[m].points[n].x * sin(Input.direction) 
        + Input_Points[m].points[n].y * cos(Input.direction);
      double vehicle_pos = (Input.distance.right - Input.distance.left)/2;
      lane_y = vehicle_pos + lane_y;
      
      for (int i = 0; i < Lane_Num/2; ++i) {
        if(fabs(lane_y) < vehicle_radius_ * Scale * (i+0.5)) {
          int k;
          (lane_y > 0) ? k = i : k = -i ;
          invailed_lane[Lane_Num/2 + k]++;
        }
      }
    }
  }
  
  for (int i = 0; i < invailed_lane.size(); i++) {
    cout << "Lane "<<i<<" has "<<invailed_lane[i] << " points" <<endl;
  }
}

void SpeedManager::filterPointCloud(std::vector<sensor_msgs::PointCloud>& Input_Points) {
  sensor_msgs::PointCloud all_in_one;
  all_in_one.header.frame_id = base_frame_;

  for (int i = 0; i < Input_Points.size(); i++) {
    if(Input_Points[i].points.empty()) continue;
    for (int j = 0; j < Input_Points[i].points.size(); j++) {
      geometry_msgs::Point32 point;
      point.x = Input_Points[i].points[j].x;
      point.y = Input_Points[i].points[j].y;
      point.z = 0;
      all_in_one.points.push_back(point);
    }
  }

  // cout<<" Origin PointCloud Size " << all_in_one.points.size() <<endl;

  sensor_msgs::PointCloud2 pointcloud_2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_out (new pcl::PointCloud<pcl::PointXYZ>);


  sensor_msgs::convertPointCloudToPointCloud2(all_in_one,pointcloud_2);
  pcl::fromROSMsg(pointcloud_2,*pointcloud_in);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (pointcloud_in);
  sor.setLeafSize (0.2, 0.2, 10);
  sor.filter (*pointcloud_out);

  pcl::toROSMsg(*pointcloud_out,pointcloud_2);
  sensor_msgs::convertPointCloud2ToPointCloud(pointcloud_2,all_in_one);

  // cout<<" Filtered PointCloud Size " << all_in_one.points.size() <<endl;

  //pointcloud_pub.publish(all_in_one);
}
  

int SpeedManager::makeDecision(std::vector<sensor_msgs::PointCloud> Input_Points) {

  int inrange_points = -1;
  double lookahead_index = 4;

  for (int i = 0; i < Input_Points.size(); i++) {
    if(Input_Points[i].points.empty()) continue;
    for (int j = 0; j < Input_Points[i].points.size(); j++) {
      if(fabs(Input_Points[i].points[j].y) > radius_scale_ * vehicle_radius_) continue;
      if(cmd_vel_.linear.x > 0) {
        if(Input_Points[i].points[j].x > lookahead_index * lookahead_time_ * max_speed_ 
          || Input_Points[i].points[j].x < radius_scale_ * vehicle_radius_) continue;
      } else {
        continue;
      }
      inrange_points++;
    }
    
  }

  if(inrange_points == -1) return inrange_points;
  inrange_points /= 100;
  return inrange_points;
}


int SpeedManager::findVehicleState(std::vector<sensor_msgs::PointCloud> Input_Points) {
  int empty_ct = 0;
  sensor_msgs::PointCloud all_in_one;
  all_in_one.header.frame_id = base_frame_;

  for (int i = 0; i < Input_Points.size(); i++) {
    if(Input_Points[i].points.empty()) {
      empty_ct++;
    } else {
      for (int j = 0; j < Input_Points[i].points.size(); j++) {
        geometry_msgs::Point32 point;
        point.x = Input_Points[i].points[j].x;
        point.y = Input_Points[i].points[j].y;
        all_in_one.points.push_back(point);
      }
    }
  }

  //pointcloud_pub.publish(all_in_one);
  return empty_ct;
}

void SpeedManager::publishPathParticle(std::vector<sensor_msgs::PointCloud> Input_Points, cv::Mat& Output_Grid){

  sensor_msgs::PointCloud front_particle;
  front_particle.header.frame_id = base_frame_;

  bool isUnsafe = false;

  int length_start,length_end;
  int width_start,width_end;

  length_start = -10*max_speed_;
  length_end = -length_start;
  width_start = -10*max_rotation_;
  width_end = -width_start;



  int map_width = ceil(fabs(width_end - width_start))+1;
  int map_height = ceil(fabs(length_end - length_start))+1;

  Output_Grid = Mat::ones(map_height,map_width,CV_8UC3) * 255;



  for (int i = length_start; i <= length_end; i++) {
    for (int j = width_start; j <= width_end; j++) {
      isUnsafe = false;
      for (int m = 0; m < Input_Points.size(); m++) {
        if(Input_Points[m].points.empty()) continue;
        for (int n = 0; n < Input_Points[m].points.size(); n++) {
          double dis_x = Input_Points[m].points[n].x - i*vehicle_radius_;
          double dis_y = Input_Points[m].points[n].y - j*vehicle_radius_;
          double dis_point = hypot(dis_x,dis_y);
          if(dis_point < vehicle_radius_) {
            isUnsafe = true;
            continue;
          }
        }
      }

      if(!isUnsafe) {
        geometry_msgs::Point32 point;
        point.x = i*vehicle_radius_;
        point.y = j*vehicle_radius_;
        front_particle.points.push_back(point);
      } else {
        Output_Grid.at<Vec3b>(length_end+i,width_end+j)[1] = 255;
        // Output_Grid.at<Vec3b>(map_height+1+i,map_width+1+j)[1] = i;
        // Output_Grid.at<Vec3b>(map_height+1+i,map_width+1+j)[2] = j;
      }
    }
  }

  //pointcloud_pub.publish(front_particle);
  
}

void SpeedManager::findClearPath(cv::Mat& Input_Grid, geometry_msgs::Twist& Output_Cmd) {
    int robot_row = ceil(Input_Grid.rows/2);
    int robot_col = ceil(Input_Grid.cols/2);

    Input_Grid.at<Vec3b>(robot_row,robot_col)[0] = 0;
    Input_Grid.at<Vec3b>(robot_row,robot_col)[1] = 0;
    Input_Grid.at<Vec3b>(robot_row,robot_col)[2] = 255;

    double speed    = Output_Cmd.linear.x;
    double rotation = Output_Cmd.angular.z;

    int move_pixle  = speed/max_speed_ * (Input_Grid.rows/2);
    int shift_pixle = 2 * rotation/max_rotation_ * (Input_Grid.cols/2);
    if(fabs(rotation/max_rotation_)<=0.5) {
      Input_Grid.at<Vec3b>(robot_row+move_pixle,robot_col+shift_pixle)[0] = 0;
      Input_Grid.at<Vec3b>(robot_row+move_pixle,robot_col+shift_pixle)[1] = 0;
      Input_Grid.at<Vec3b>(robot_row+move_pixle,robot_col+shift_pixle)[2] = 255;
    } else {
      shift_pixle = 2 * (rotation/max_rotation_ - sign(rotation)*0.5) 
        * (Input_Grid.rows/2); 
      move_pixle  = sign(rotation) * sign(speed) * speed/max_speed_ * (Input_Grid.cols/2);
      shift_pixle = fabs(shift_pixle);

      if(speed >= 0) {
        Input_Grid.at<Vec3b>(robot_row-shift_pixle+Input_Grid.rows/2,robot_col+move_pixle)[0] = 0;
        Input_Grid.at<Vec3b>(robot_row-shift_pixle+Input_Grid.rows/2,robot_col+move_pixle)[1] = 0;
        Input_Grid.at<Vec3b>(robot_row-shift_pixle+Input_Grid.rows/2,robot_col+move_pixle)[2] = 255;
      } else {
        Input_Grid.at<Vec3b>(shift_pixle,robot_col+move_pixle)[0] = 0;
        Input_Grid.at<Vec3b>(shift_pixle,robot_col+move_pixle)[1] = 0;
        Input_Grid.at<Vec3b>(shift_pixle,robot_col+move_pixle)[2] = 255;
      }
    }


    // searchPath(Input_Grid,
    //   robot_row,robot_col,
    //   robot_row+move_pixle,robot_col+shift_pixle);

    static int ref_timer = 0;
    //shiftLine(Input_Grid,Output_Cmd,ref_timer);


    cv::rotate(Input_Grid, Input_Grid, cv::ROTATE_180);
    cv::imshow("Map",Input_Grid);
    cv::resizeWindow("Map",600,600);
    cv::waitKey(1);
}

void SpeedManager::shiftLine(cv::Mat& Input_Grid,
  geometry_msgs::Twist& Output_Cmd,int& Ref_Timer) {

  if(Ref_Timer != 0) { 
    if(Ref_Timer > 0) {
      Output_Cmd.angular.z += 0.3;
      cout << "Turning Left" << endl;
      Ref_Timer-=1;
     } else { 
      Output_Cmd.angular.z -= 0.3;
      cout << "Turning Right" << endl;
      Ref_Timer+=1;
    }
  } else {
    cout << "Seraching" << endl;
    int unit_time = 30;
    int search_col = (Input_Grid.cols/2);
    int pixle_r_ct = 0;
    int pixle_l_ct = 0;
    for (int i = 0; i <= (Input_Grid.cols/2); i++) {
      for (int row = 0; row < Input_Grid.rows;  row++) {
        if(Input_Grid.at<Vec3b>(row,search_col+i)[1] == 0) pixle_r_ct++;
        if(Input_Grid.at<Vec3b>(row,search_col-i)[1] == 0) pixle_l_ct++;
      }
      if(pixle_r_ct == Input_Grid.rows) {
        Ref_Timer = i * unit_time;
        return;
      }
      if(pixle_l_ct == Input_Grid.rows) {
        Ref_Timer = -i * unit_time;
        return;
      }
      pixle_r_ct = 0;
      pixle_l_ct = 0;
    }
  }

}


void SpeedManager::searchPath(cv::Mat& Input_Grid,
  int Start_Row,int Start_Col,int End_Row,int End_Col) {

  bool isFoundPath = false;
  bool isOut = false;
  int current_row = Start_Row;
  int current_col = Start_Col;
  int shift_pixle = 0;

  if(Input_Grid.at<Vec3b>(End_Row,End_Col)[1] == 255) {
    cout<<"NO"<<endl;
    return;
  }


  while(!isFoundPath) {
    
    if(current_col != End_Col) current_col += sign(End_Col-current_col) * 1;
    if(current_row != End_Row) current_row += sign(End_Row-current_row) * 1;

    while(Input_Grid.at<Vec3b>(current_row,current_col)[1] == 255 || isOut) {
      current_col++;
      shift_pixle++;
      if(shift_pixle>Input_Grid.rows && shift_pixle>Input_Grid.cols) isOut = true;
      // cout<<Start_Row<<","<<Start_Col<<"  "
      // <<End_Row<<","<<End_Col<<"  "
      // <<current_row<<","<<current_col<<endl;
    }

    if(current_row>Input_Grid.rows || current_row < 0) break;
    if(current_col>Input_Grid.cols || current_col < 0) break;

    Input_Grid.at<Vec3b>(current_row,current_col)[2] = 255;

    if(current_row == End_Row) isFoundPath = true;
    //isOut = false;
  }

  Input_Grid.at<Vec3b>(End_Row,End_Col)[0] = 0;
  Input_Grid.at<Vec3b>(End_Row,End_Col)[1] = 0;
  Input_Grid.at<Vec3b>(End_Row,End_Col)[2] = 255;
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

  double max_acc_rt = 1;
  double max_acc_loop_linear = max_acc_rt/ROS_RATE_HZ;
  double max_acc_loop_angular = 2 * max_acc_rt/ROS_RATE_HZ;

  if (input_cmd.linear.x > last_cmd_linear && input_cmd.linear.x > 0) {
    input_cmd.linear.x = last_cmd_linear + max_acc_loop_linear;
  }
  else if (input_cmd.linear.x < last_cmd_linear && input_cmd.linear.x < 0) {
    input_cmd.linear.x = last_cmd_linear - max_acc_loop_linear;
  }

  if (input_cmd.angular.z > last_cmd_anglar && input_cmd.angular.z >= 0) {
    input_cmd.angular.z = last_cmd_anglar + max_acc_loop_angular;
  }
  else if (input_cmd.angular.z < last_cmd_anglar && input_cmd.angular.z < 0) {
    input_cmd.angular.z = last_cmd_anglar - max_acc_loop_angular; 
  }


  last_cmd_linear = input_cmd.linear.x;
  last_cmd_anglar = input_cmd.angular.z;

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
     
  // if (last_cmd_linear == nav_speed_ && last_cmd_anglar == nav_rotation_) {
  //   if (stuck_ct > stuck_limit) isNav_ = false;
  //   else stuck_ct++;
  // } else {
  //   last_cmd_linear = nav_speed_;
  //   last_cmd_anglar = nav_rotation_;
  //   stuck_ct = 0;
  // }

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