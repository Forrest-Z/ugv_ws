#ifndef LIB_OBSTACLEMANAGER_H
#define LIB_OBSTACLEMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using std::vector;
using std::cout;
using std::endl;
using std::isnan;
using std::string;

const int ROS_RATE_HZ = 100;
const double PI = 3.14159265359;

class ObstacleManager
{
public:
  ObstacleManager();
  ~ObstacleManager();

  void Manager();
  void Mission();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/
  ros::Publisher map_obs_pub;
  ros::Publisher odom_pub;

  /** Subscribers **/
  ros::Subscriber map_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber pointcloud_sub;
  ros::Subscriber scan_points_sub;
  ros::Subscriber rviz_click_sub;
  ros::Subscriber odom_sub;

  /** Parameters **/
  tf::TransformListener map_base_listener;

  /** Flags **/
  bool isMapSave_;
  bool isUseSim_;
  /** Variables **/
  laser_geometry::LaserProjection projector;
  nav_msgs::OccupancyGrid static_map_;
  nav_msgs::MapMetaData static_map_info_;
  geometry_msgs::Point32 robot_in_map_;
  sensor_msgs::PointCloud pointcloud_base_;
  sensor_msgs::PointCloud pointcloud_scan_;
  sensor_msgs::PointCloud pointcloud_lidar_;
  sensor_msgs::PointCloud pointcloud_rviz_;

  tf::Transform map_to_base_;
  double inflation_;

  /** Functions **/
  bool updateVehicleInMap();
  void publishMapObstacle();
  void publishScanObstacle();
  void publishLidarObstacle();
  void publishRvizObstacle();

  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input);
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& input);
  void scanpoints_callback(const sensor_msgs::PointCloud::ConstPtr& input);
  void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void clickpoint_callback(const geometry_msgs::PointStamped::ConstPtr& input);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& input);

  
};

#endif
