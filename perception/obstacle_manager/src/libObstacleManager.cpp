#include "libObstacleManager.h"

ObstacleManager::ObstacleManager():pn("~")
{ 
  map_sub = n.subscribe("/map",1, &ObstacleManager::map_callback,this);
  scan_1_sub = n.subscribe("/scan_1",1, &ObstacleManager::scan_1_callback,this);
  scan_2_sub = n.subscribe("/scan_22",1, &ObstacleManager::scan_2_callback,this);
  pointcloud_sub = n.subscribe("/rslidar_points",1, &ObstacleManager::pointcloud_callback,this);
  scan_points_sub = n.subscribe("/scan_points1",1, &ObstacleManager::scanpoints_callback,this);

  odom_sub = n.subscribe("/odom_1123",10, &ObstacleManager::OdomCallback,this);

  rviz_click_sub = n.subscribe("/clicked_point",1, &ObstacleManager::clickpoint_callback,this);

  map_obs_pub = n.advertise<sensor_msgs::PointCloud> ("/map_obs_points", 1);
  odom_pub = n.advertise<nav_msgs::Odometry>("/tf_odom", 50);

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
  publishLidarObstacle();
  publishRvizObstacle();

  pointcloud_base_.header.frame_id = "/base_link";
  map_obs_pub.publish(pointcloud_base_);
  pointcloud_base_.points.clear();
}


bool ObstacleManager::updateVehicleInMap() {
  tf::StampedTransform stampedtransform;
  try {
    map_base_listener.lookupTransform("/map", "/base_link",  
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
  if(pointcloud_scan_1_.points.size()==0) return;
  for (int i = 0; i < pointcloud_scan_1_.points.size(); ++i) {
    pointcloud_scan_1_.points[i].z = 1;
    // pointcloud_scan_2_.points[i].z = 1;
    // pointcloud_scan_1_.points[i].x -= 0.5;
    // pointcloud_scan_2_.points[i].x -= 0.5;
    pointcloud_base_.points.push_back(pointcloud_scan_1_.points[i]);
    // pointcloud_base_.points.push_back(pointcloud_scan_2_.points[i]);
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


void ObstacleManager::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input) {
    cout << "Map Received, size " << input->info.width << "x" << input->info.height << endl;

    static_map_ = *input;
    static_map_info_ = input->info;
  }

  
  void ObstacleManager::scan_1_callback(const sensor_msgs::LaserScan::ConstPtr& input) {
    // projector.projectLaser(*input, pointcloud_scan_1_);
    double angle_increment = input -> angle_increment;
    double angle_current = 0;
    pointcloud_scan_1_.points.clear();
    geometry_msgs::Point32 point;
    point.z = 1;
    for (int i = 0; i < input->ranges.size(); ++i) {
      point.x = input->ranges[i] * cos(angle_current) + 0.5;
      point.y = input->ranges[i] * sin(angle_current);
      angle_current += angle_increment;
      if(input->ranges[i] < 0.1 || point.x < 0.5) continue;
      pointcloud_scan_1_.points.push_back(point);
    }
  }

  void ObstacleManager::scan_2_callback(const sensor_msgs::LaserScan::ConstPtr& input) {
    // projector.projectLaser(*input, pointcloud_scan_2_);


    // double angle_increment = input -> angle_increment;
    // double angle_current = 0;
    // pointcloud_scan_2_.points.clear();
    // geometry_msgs::Point32 point;
    // point.z = 1;
    // for (int i = 0; i < input->ranges.size(); ++i) {
    //   point.x = input->ranges[i] * cos(angle_current);
    //   point.y = input->ranges[i] * sin(angle_current);
    //   angle_current += angle_increment;
    //   pointcloud_scan_2_.points.push_back(point);
    // }
  }

  void ObstacleManager::scanpoints_callback(const sensor_msgs::PointCloud::ConstPtr& input) {
    pointcloud_lidar_ = *input;
  }


  void ObstacleManager::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    pointcloud_lidar_.points.clear();
    double x_fix = 0.08;
    double z_fix = 0;//0.64;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::fromROSMsg(*input, *cloud);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setInputCloud (cloud);
    approximate_voxel_filter.setLeafSize (0.1f, 0.1f, 0.1f);
    approximate_voxel_filter.filter (*cloud_filtered);

    for (int i = 0; i < cloud_filtered->points.size(); ++i) {
      geometry_msgs::Point32 point;
      point.x = cloud_filtered->points[i].x - x_fix;
      point.y = cloud_filtered->points[i].y;
      point.z = cloud_filtered->points[i].z - z_fix;

      double range = hypot(point.x, point.y);
      double height = point.z;
      double width = point.y;
      
      if(height < -0.8) continue;
      if(height > 0) continue;
      if(range < 0.1) continue;
      if(range > 10) continue;
      if(fabs(width) > 5) continue;
      if(point.x < 0 && fabs(point.y) < 0.3 && range < 1) continue;
      point.z = 1;
      pointcloud_lidar_.points.push_back(point);
    }


    sensor_msgs::PointCloud2 output;
    sensor_msgs::convertPointCloudToPointCloud2(pointcloud_lidar_,output);

    pcl::fromROSMsg(output, *cloud);

    approximate_voxel_filter.setInputCloud (cloud);
    approximate_voxel_filter.setLeafSize (0.1f, 0.1f, 0.1f);
    approximate_voxel_filter.filter (*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, output);
    sensor_msgs::convertPointCloud2ToPointCloud(output,pointcloud_lidar_);

  }

  void ObstacleManager::clickpoint_callback(const geometry_msgs::PointStamped::ConstPtr& input) {
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

  void ObstacleManager::OdomCallback(const nav_msgs::Odometry::ConstPtr& input) {
    geometry_msgs::TransformStamped odom_trans;

    tf::Transform map_to_odom;
    tf::Transform odom_to_base;
    tf::Transform base_to_map;

    if(true){
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/odom";
      odom_trans.child_frame_id = "/base_link";

      odom_trans.transform.translation.x = input->pose.pose.position.x;
      odom_trans.transform.translation.y = input->pose.pose.position.y;
      odom_trans.transform.translation.z = input->pose.pose.position.z;
      odom_trans.transform.rotation = input->pose.pose.orientation;
      transformMsgToTF(odom_trans.transform,odom_to_base);

      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/map";
      odom_trans.child_frame_id = "/odom";

      odom_trans.transform.translation.x = 0;
      odom_trans.transform.translation.y = 0;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,1.57);
      transformMsgToTF(odom_trans.transform,map_to_odom);
    } else {
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/odom";
      odom_trans.child_frame_id = "/base_link";

      odom_trans.transform.translation.x = 0;
      odom_trans.transform.translation.y = 0;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
      transformMsgToTF(odom_trans.transform,odom_to_base);

      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/map";
      odom_trans.child_frame_id = "/odom";

      odom_trans.transform.translation.x = input->pose.pose.position.x;
      odom_trans.transform.translation.y = input->pose.pose.position.y;
      odom_trans.transform.translation.z = input->pose.pose.position.z;
      odom_trans.transform.rotation = input->pose.pose.orientation;
      transformMsgToTF(odom_trans.transform,map_to_odom);      
    }

    base_to_map = map_to_odom * odom_to_base;


    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();;
    odom.header.frame_id = "/map";

    //set the position
    odom.pose.pose.position.x = base_to_map.getOrigin().x();
    odom.pose.pose.position.y = base_to_map.getOrigin().y();
    odom.pose.pose.position.z = base_to_map.getOrigin().z();
    odom.pose.pose.orientation.x = base_to_map.getRotation().x();
    odom.pose.pose.orientation.y = base_to_map.getRotation().y();
    odom.pose.pose.orientation.z = base_to_map.getRotation().z();
    odom.pose.pose.orientation.w = base_to_map.getRotation().w();
    //publish the message
    odom_pub.publish(odom);
  }
