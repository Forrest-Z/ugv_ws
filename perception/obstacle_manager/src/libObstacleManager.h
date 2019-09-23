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

const int ROS_RATE_HZ = 50;

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

  /** Subscribers **/
  ros::Subscriber map_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber pointcloud_sub;

  /** Parameters **/
  tf::TransformListener map_base_listener;

  /** Flags **/
  bool isMapSave_;
  /** Variables **/
  laser_geometry::LaserProjection projector;
  nav_msgs::OccupancyGrid static_map_;
  nav_msgs::MapMetaData static_map_info_;
  geometry_msgs::Point32 robot_in_map_;
  sensor_msgs::PointCloud pointcloud_base_;
  sensor_msgs::PointCloud pointcloud_scan_;
  sensor_msgs::PointCloud pointcloud_lidar_;

  tf::Transform map_to_base_;
  double inflation_;

  /** Functions **/

  bool updateVehicleInMap();

  void publishMapObstacle();
  void publishScanObstacle();
  void publishLidarObstacle();

  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input) {
    cout << "Map Received, size " << input->info.width << "x" << input->info.height << endl;

    static_map_ = *input;
    static_map_info_ = input->info;
  }

  
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& input) {
    projector.projectLaser(*input, pointcloud_scan_);
  }


  void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    pointcloud_lidar_.points.clear();
    double x_fix = 0.08;
    double z_fix = 0;//0.64;

    double ground_height = 0.1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::fromROSMsg(*input, *cloud);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setInputCloud (cloud);
    approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
    approximate_voxel_filter.filter (*cloud_filtered);

    for (int i = 0; i < cloud_filtered->points.size(); ++i) {
      geometry_msgs::Point32 point;
      point.x = cloud_filtered->points[i].x - x_fix;
      point.y = cloud_filtered->points[i].y;
      point.z = cloud_filtered->points[i].z - z_fix;

      double range = hypot(point.x, point.y);
      double height = point.z;
      if(height < -0.1) continue;
      if(height > 1) continue;
      if(range < 0.5) continue;
      pointcloud_lidar_.points.push_back(point);
    }

  }



  
};

#endif
