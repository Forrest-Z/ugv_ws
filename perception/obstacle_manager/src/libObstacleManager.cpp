#include "libObstacleManager.h"

ObstacleManager::ObstacleManager():pn("~")
{ 
  isUseSim_ = true;
  string odom_topic;
  (isUseSim_) ? odom_topic = "/odom" : odom_topic = "/ekf_2/odometry/filtered";

  map_sub = n.subscribe("/map",1, &ObstacleManager::map_callback,this);
  scan_sub = n.subscribe("/scan",1, &ObstacleManager::scan_callback,this);
  odom_sub = n.subscribe(odom_topic,1, &ObstacleManager::odom_callback,this); 
  pointcloud_sub = n.subscribe("/rslidar_points",1, &ObstacleManager::pointcloud_callback,this);

  map_obs_pub = n.advertise<sensor_msgs::PointCloud> ("/map_obs_points", 1);

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

    try {
      map_base_listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), map_base_transform);
    }
    catch (tf::TransformException ex) {
      cout << "Waiting For TF" << endl;
      continue;
    }

    robot_in_map_.x = map_base_transform.getOrigin().x();
    robot_in_map_.y = map_base_transform.getOrigin().y();
    robot_in_map_.z = tf::getYaw(map_base_transform.getRotation());

    Mission();
  }
}

void ObstacleManager::Mission() {
  publishMapObstacle();
  publishScanObstacle();
  publishLidarObstacle();

  pointcloud_base_.header.frame_id = "/base_link";
  map_obs_pub.publish(pointcloud_base_);
  pointcloud_base_.points.clear();
}

void ObstacleManager::publishMapObstacle() {
  geometry_msgs::Point32 point;
  sensor_msgs::PointCloud pointcloud_map;

  tf::TransformListener transform_listener;
  tf::StampedTransform transform;

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
    if(hypot(pointcloud_scan_.points[i].x,pointcloud_scan_.points[i].y) < 0.5) continue;
    pointcloud_scan_.points[i].z = 1;
    for (int j = 0; j < 16; ++j) {
      pointcloud_base_.points.push_back(pointcloud_scan_.points[i]);
    }
    
  }
}

void ObstacleManager::publishLidarObstacle() {
  if(pointcloud_lidar_.points.size()==0) return;
  for (int i = 0; i < pointcloud_lidar_.points.size(); ++i) {
    if(isnan(pointcloud_lidar_.points[i].x) || isnan(pointcloud_lidar_.points[i].y) ) continue;
    pointcloud_lidar_.points[i].z = 1;
    pointcloud_base_.points.push_back(pointcloud_lidar_.points[i]);
  }
}
