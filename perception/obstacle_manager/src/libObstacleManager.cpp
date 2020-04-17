#include "libObstacleManager.h"

ObstacleManager::ObstacleManager():pn("~")
{ 
  pn.param<string>("robot_id", robot_id_, "");

  map_sub = n.subscribe("/map",1, &ObstacleManager::MapCallback,this);
  scan_sub = n.subscribe(robot_id_ + "/scan",1, &ObstacleManager::SacnCallback,this);
  rviz_click_sub = n.subscribe("/clicked_point",1, &ObstacleManager::ClickpointCallback,this);

  map_obs_pub = n.advertise<sensor_msgs::PointCloud> (robot_id_ + "/map_obs_points", 1);

  isMapSave_ = false;

  inflation_ = 1;
}


ObstacleManager::~ObstacleManager(){
}

void ObstacleManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
    if(!updateVehicleInMap()) continue;
    Mission();
  }
}

void ObstacleManager::Mission() {
  publishMapObstacle();
  publishScanObstacle();
  publishRvizObstacle();

  pointcloud_base_.header.frame_id = robot_id_ + "/base_link";
  map_obs_pub.publish(pointcloud_base_);
  pointcloud_base_.points.clear();
}


bool ObstacleManager::updateVehicleInMap() {
  tf::StampedTransform stampedtransform;
  try {
    map_base_listener.lookupTransform("/map", robot_id_ + "/base_link",  
                             ros::Time(0), stampedtransform);
  }
  catch (tf::TransformException ex) {
    cout << "Waiting For TF OBS" << endl;
    return false;
  }
  robot_in_map_.x = stampedtransform.getOrigin().x();
  robot_in_map_.y = stampedtransform.getOrigin().y();
  robot_in_map_.z = tf::getYaw(stampedtransform.getRotation());

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
  transformMsgToTF(temp_trans.transform,map_to_base_);
  map_to_base_ = map_to_base_.inverse();

  return true;
}



void ObstacleManager::publishMapObstacle() {
  geometry_msgs::Point32 point;
  sensor_msgs::PointCloud pointcloud_map;

  pointcloud_map.header.frame_id = "/map";
  
  double window_size = 10;

  int search_unit = window_size/static_map_info_.resolution;
  int vehicle_in_grid_x = (robot_in_map_.x - static_map_info_.origin.position.x) 
  / static_map_info_.resolution;
  int vehicle_in_grid_y = (robot_in_map_.y - static_map_info_.origin.position.y) 
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

  int point_gap = 3;

  for (int i = search_row_begin; i < search_row_end; i+=point_gap) {
    for (int j = search_col_begin; j < search_col_end; j+=point_gap) {
      if(static_map_.data[i+j*static_map_info_.width] == 0) continue;
      point.x = i * static_map_info_.resolution + static_map_info_.origin.position.x;
      point.y = j * static_map_info_.resolution + static_map_info_.origin.position.y;
      pointcloud_map.points.push_back(point);
    }
  }
  double transform_m[4][4];
  double mv[12];
  map_to_base_.getBasis().getOpenGLSubMatrix(mv);
  tf::Vector3 origin = map_to_base_.getOrigin();

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

  pointcloud_base_ = pointcloud_map;
  for (int i = 0; i < pointcloud_map.points.size(); ++i) {
    pointcloud_base_.points[i].x 
      = pointcloud_map.points[i].x * transform_m[0][0] 
      + pointcloud_map.points[i].y * transform_m[0][1]
      + pointcloud_map.points[i].z * transform_m[0][2] 
      + transform_m[0][3];
    pointcloud_base_.points[i].y 
      = pointcloud_map.points[i].x * transform_m[1][0] 
      + pointcloud_map.points[i].y * transform_m[1][1] 
      + pointcloud_map.points[i].z * transform_m[1][2] 
      + transform_m[1][3];   
    pointcloud_base_.points[i].z = 10;
  }
}


void ObstacleManager::publishScanObstacle() {
  if(pointcloud_scan_.points.size()==0) return;
  for (int i = 0; i < pointcloud_scan_.points.size(); ++i) {
    pointcloud_scan_.points[i].z = 1;
    // pointcloud_scan_2_.points[i].z = 1;
    // pointcloud_scan_1_.points[i].x -= 0.5;
    // pointcloud_scan_2_.points[i].x -= 0.5;
    pointcloud_base_.points.push_back(pointcloud_scan_.points[i]);
    // pointcloud_base_.points.push_back(pointcloud_scan_2_.points[i]);
  }
}

void ObstacleManager::publishRvizObstacle() {
  if(pointcloud_rviz_.points.size()==0) return;

  geometry_msgs::Point32 point_in_local;

  double transform_m[4][4];
  double mv[12];
  map_to_base_.getBasis().getOpenGLSubMatrix(mv);
  tf::Vector3 origin = map_to_base_.getOrigin();

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

  for (int i = 0; i < pointcloud_rviz_.points.size(); ++i){
    point_in_local.x 
      = pointcloud_rviz_.points[i].x * transform_m[0][0] 
      + pointcloud_rviz_.points[i].y * transform_m[0][1]
      + pointcloud_rviz_.points[i].z * transform_m[0][2] 
      + transform_m[0][3];
    point_in_local.y 
      = pointcloud_rviz_.points[i].x * transform_m[1][0] 
      + pointcloud_rviz_.points[i].y * transform_m[1][1] 
      + pointcloud_rviz_.points[i].z * transform_m[1][2] 
      + transform_m[1][3];   
    point_in_local.z = 1;
    for (int j = 0; j < 16; ++j) {
      pointcloud_base_.points.push_back(point_in_local);
    }
  }

}


void ObstacleManager::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input) {
    cout << "Map Received, size " << input->info.width << "x" << input->info.height << endl;

    static_map_ = *input;
    static_map_info_ = input->info;
  }

  
  void ObstacleManager::SacnCallback(const sensor_msgs::LaserScan::ConstPtr& input) {
    projector.projectLaser(*input, pointcloud_scan_);
    // double angle_increment = input -> angle_increment;
    // double angle_current = 0;
    // pointcloud_scan_1_.points.clear();
    // geometry_msgs::Point32 point;
    // point.z = 1;
    // for (int i = 0; i < input->ranges.size(); ++i) {
    //   point.x = input->ranges[i] * cos(angle_current) + 0.5;
    //   point.y = input->ranges[i] * sin(angle_current);
    //   angle_current += angle_increment;
    //   if(input->ranges[i] < 0.1 || point.x < 0.5) continue;
    //   pointcloud_scan_1_.points.push_back(point);
    // }
    // scan_pub.publish(input);
  }


  void ObstacleManager::ClickpointCallback(const geometry_msgs::PointStamped::ConstPtr& input) {
    geometry_msgs::Point32 point;
    cout << "Click Point Received , current total " << pointcloud_rviz_.points.size()/32 << endl;
    if(pointcloud_rviz_.points.size() > 320) {
      pointcloud_rviz_.points.clear();
    }
    double obs_radius = 0.15; 
    for (double i = 0; i < 2*PI; i+=0.2) {
      point.x = input->point.x + obs_radius*cos(i);
      point.y = input->point.y + obs_radius*sin(i);
      pointcloud_rviz_.points.push_back(point);
    }

  }
