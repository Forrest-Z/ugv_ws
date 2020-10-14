#include "libObstacleManager.h"

ObstacleManager::ObstacleManager():pn("~") { 
  n.param<string>("robot_id", robot_id_, "");
  n.param<string>("community_id", community_id_, "2");

  map_sub = n.subscribe("/map",1, &ObstacleManager::MapCallback,this);
  scan_sub = n.subscribe("/scan",1, &ObstacleManager::SacnCallback,this);
  lidar_sub = n.subscribe("/rslidar_points",1, &ObstacleManager::LidarCallback,this);

  rviz_click_sub = n.subscribe("/clicked_point",1, &ObstacleManager::ClickpointCallback,this);

  map_obs_pub = n.advertise<sensor_msgs::PointCloud> ("/map_obs_points", 1);
  scan_str_pub = n.advertise<std_msgs::String> ("from_robot_lidar", 1);
  convert_scan_pub = n.advertise<sensor_msgs::LaserScan> ("lidar_scan", 1);

  isMapSave_ = false;

  inflation_ = 1;
  mqtt_timer_ = ros::Time::now();
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
  publishLidarObstacle();
  publishRvizObstacle();
  pointcloud_base_.header.frame_id = "/base_link";
  map_obs_pub.publish(pointcloud_base_);
  pointcloud_base_.points.clear();
}


bool ObstacleManager::updateVehicleInMap() {
  tf::StampedTransform stampedtransform;
  try {
    map_base_listener.lookupTransform("/map","/base_link",  
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
  temp_trans.child_frame_id = "/base_link";

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

  if(static_map_.data.empty()) return;


  double window_size = 45;

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

void ObstacleManager::publishLidarObstacle() {
  double max_height = 0.2;
  double min_height = -0.1;

  double max_range = 10;
  double min_range = 0.1;

  int ranges_size = 720;

  if(pointcloud_lidar_.data.empty()) return;

  sensor_msgs::LaserScan scan_from_pointcloud;
  scan_from_pointcloud.header.stamp = ros::Time::now();
  scan_from_pointcloud.header.frame_id = "/base_link";
  scan_from_pointcloud.range_max = max_range;
  scan_from_pointcloud.range_min = min_range;
  scan_from_pointcloud.angle_min = -PI;
  scan_from_pointcloud.angle_max = PI;
  scan_from_pointcloud.angle_increment = (scan_from_pointcloud.angle_max-scan_from_pointcloud.angle_min)/ranges_size;
  scan_from_pointcloud.ranges.assign(ranges_size, scan_from_pointcloud.range_max + 1);

  for (sensor_msgs::PointCloud2ConstIterator<float>
          iter_x(pointcloud_lidar_, "x"), iter_y(pointcloud_lidar_, "y"), iter_z(pointcloud_lidar_, "z");
          iter_x != iter_x.end();
          ++iter_x, ++iter_y, ++iter_z) { 
    double range = hypot(*iter_x, *iter_y);
    double angle = atan2(*iter_y, *iter_x);
    if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z)) continue;
    if (*iter_z > max_height || *iter_z < min_height) continue;
    if (range > max_range) continue;
    geometry_msgs::Point32 point;
    point.x = *iter_x;
    point.y = *iter_y;
    point.z = 1;
    pointcloud_base_.points.push_back(point);
    
    if (angle < scan_from_pointcloud.angle_min || angle > scan_from_pointcloud.angle_max) continue;
    int index = (angle - scan_from_pointcloud.angle_min) / scan_from_pointcloud.angle_increment;
    if (range < scan_from_pointcloud.ranges[index]) {
      scan_from_pointcloud.ranges[index] = range;
    }
  }

  convert_scan_pub.publish(scan_from_pointcloud);

  if((ros::Time::now() - mqtt_timer_).toSec() < 0.1) return;
  mqtt_timer_ = ros::Time::now();
  string delimiter = "$";
  string range_delimiter = "#";

  int id_int = stoi(robot_id_.substr(6,2));
  int angle_size = scan_from_pointcloud.ranges.size();
  int angle_min = static_cast<int>(scan_from_pointcloud.angle_min * 100);
  int angle_max = static_cast<int>(scan_from_pointcloud.angle_max * 100);

  string head_str = "GPMAPD";
  string source_str = "robot";
  string community_str = community_id_;
  string id_str = to_string(id_int);
  string status_str = "ok";
  string time_str = to_string(ros::Time::now().toNSec()).substr(0,13);
  string type_str = "lidar";
  string task_str = "single";
  string min_angle_str = to_string(angle_min);
  string max_angle_str = to_string(angle_max);
  string angle_size_str = to_string(angle_size);

  string data_str;
  if(scan_from_pointcloud.ranges[0] > scan_from_pointcloud.range_max) {
    data_str +=  "nan";
  } else if (scan_from_pointcloud.ranges[0] < scan_from_pointcloud.range_min) {
    data_str +=  "0";
  } else {
    int ranges_2d = static_cast<int>(scan_from_pointcloud.ranges[0] * 100);
    data_str +=  to_string(ranges_2d);
  }

  string end_str = "HE";

  for (int i = 1; i < scan_from_pointcloud.ranges.size(); ++i) {
    if(scan_from_pointcloud.ranges[i] > scan_from_pointcloud.range_max) {
      data_str += range_delimiter + "nan";
    } else if (scan_from_pointcloud.ranges[i] < scan_from_pointcloud.range_min) {
      data_str += range_delimiter + "0";
    } else {
      int ranges_2d = static_cast<int>(scan_from_pointcloud.ranges[i] * 100);
      data_str += range_delimiter + to_string(ranges_2d);
    }

  }
  int length_raw = head_str.size() + source_str.size() + community_str.size() +
                   id_str.size() + status_str.size() +
                   time_str.size() + type_str.size() + 
                   task_str.size() + min_angle_str.size() +
                   max_angle_str.size() + angle_size_str.size() +
                   data_str.size() + end_str.size() + 13 * delimiter.size();
  length_raw += static_cast<int>(log10(static_cast<double>(length_raw)));
  length_raw ++;
  if(to_string(length_raw).size() != to_string(length_raw-1).size()) length_raw++;
  string length_str = to_string(length_raw);

  string output = head_str + delimiter + 
                  length_str + delimiter + 
                  source_str + delimiter + 
                  community_str + delimiter + 
                  id_str + delimiter + 
                  status_str + delimiter + 
                  time_str + delimiter + 
                  type_str + delimiter + 
                  task_str + delimiter + 
                  min_angle_str + delimiter + 
                  max_angle_str + delimiter +
                  angle_size_str + delimiter + 
                  data_str + delimiter + 
                  end_str;

  std_msgs::String output_msg;
  output_msg.data = output;
  scan_str_pub.publish(output_msg);
}


void ObstacleManager::publishScanObstacle() {
  if(pointcloud_scan_.points.size()==0) return;
  for (int i = 0; i < pointcloud_scan_.points.size(); ++i) {
    pointcloud_scan_.points[i].z = 1;
    // pointcloud_scan_.points[i].x += 0.25;
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

void ObstacleManager::LidarCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
  sensor_msgs::PointCloud2 temp_pointcloud2;

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  pcl::fromROSMsg(*input,*cloud); 
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud_filtered);
  pcl::toROSMsg(*cloud_filtered,temp_pointcloud2);
  
  pointcloud_lidar_ = temp_pointcloud2;
}


void ObstacleManager::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input) {
    cout << "Map Received, size " << input->info.width << "x" << input->info.height << endl;

    static_map_ = *input;
    static_map_info_ = input->info;
}


void ObstacleManager::SacnCallback(const sensor_msgs::LaserScan::ConstPtr& input) {
  projector.projectLaser(*input, pointcloud_scan_);
  if((ros::Time::now() - mqtt_timer_).toSec() < 0.1) return;
  mqtt_timer_ = ros::Time::now();

  int id_int = stoi(robot_id_.substr(6,2));

  int angle_size = input->ranges.size();
  int angle_min = static_cast<int>(input->angle_min * 100);
  int angle_max = static_cast<int>(input->angle_max * 100);

  string delimiter = "$";
  string range_delimiter = "#";

  string head_str = "GPMAPD";
  string source_str = "robot";
  string community_str = community_id_;
  string id_str = to_string(id_int);
  string status_str = "ok";
  string time_str = to_string(ros::Time::now().toNSec()).substr(0,13);
  string type_str = "lidar";
  string task_str = "single";
  string min_angle_str = to_string(angle_min);
  string max_angle_str = to_string(angle_max);
  string angle_size_str = to_string(angle_size);

  string data_str;
  if(input->ranges[0] > input->range_max) {
    data_str +=  "nan";
  } else if (input->ranges[0] < input->range_min) {
    data_str +=  "0";
  } else {
    int ranges_2d = static_cast<int>(input->ranges[0] * 100);
    data_str +=  to_string(ranges_2d);
  }

  string end_str = "HE";

  for (int i = 1; i < input->ranges.size(); ++i) {
    if(input->ranges[i] > input->range_max) {
      data_str += range_delimiter + "nan";
    } else if (input->ranges[i] < input->range_min) {
      data_str += range_delimiter + "0";
    } else {
      int ranges_2d = static_cast<int>(input->ranges[i] * 100);
      data_str += range_delimiter + to_string(ranges_2d);
    }

  }
  int length_raw = head_str.size() + source_str.size() + community_str.size() +
                   id_str.size() + status_str.size() +
                   time_str.size() + type_str.size() + 
                   task_str.size() + min_angle_str.size() +
                   max_angle_str.size() + angle_size_str.size() +
                   data_str.size() + end_str.size() + 13 * delimiter.size();
  length_raw += static_cast<int>(log10(static_cast<double>(length_raw)));
  length_raw ++;
  if(to_string(length_raw).size() != to_string(length_raw-1).size()) length_raw++;
  string length_str = to_string(length_raw);

  string output = head_str + delimiter + 
                  length_str + delimiter + 
                  source_str + delimiter + 
                  community_str + delimiter + 
                  id_str + delimiter + 
                  status_str + delimiter + 
                  time_str + delimiter + 
                  type_str + delimiter + 
                  task_str + delimiter + 
                  min_angle_str + delimiter + 
                  max_angle_str + delimiter +
                  angle_size_str + delimiter + 
                  data_str + delimiter + 
                  end_str;

  std_msgs::String output_msg;
  output_msg.data = output;
  scan_str_pub.publish(output_msg);
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
