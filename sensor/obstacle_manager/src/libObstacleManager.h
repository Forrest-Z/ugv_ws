#ifndef LIB_OBSTACLEMANAGER_H
#define LIB_OBSTACLEMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point32.h>

using std::vector;
using std::cout;
using std::endl;
using std::isnan;
using std::string;

const int ROS_RATE_HZ   = 20;

const vector<double> R11 = {4.619579111799e-01, 1.120235603441e-01, 8.797986191318e-01};
const vector<double> R12 = {-8.847258557293e-01, -1.123975717332e-02, 4.659762097605e-01};
const vector<double> R13 = {6.208903689347e-02, -9.936419827013e-01, 9.391784554130e-02};
const vector<double> T1  = {6.566721252215e-02, 1.507875013048e-01, -3.012084046145e-01 };
//const vector<double> T1  = {2.566721252215e-02, 1.607875013048e-01, -3.012084046145e-01 };

const vector<double> R21 = {0.911020853031573, 0.00519415403124045, 0.412327571362292};
const vector<double> R22 = {-0.00168139473597834, 0.999959143617621, -0.00888166691871711};
const vector<double> R23 = {-0.412356857895401, 0.00739809836464467, 0.910992365438492};
const vector<double> T2  = {-10.6632863659008, 12.5020142707178, 2.33286297823336};

const vector<double> R31 = {0.911020853031573,-0.00168139473597833,-0.412356857895401};
const vector<double> R32 = {0.00519415403124034,0.999959143617621,0.00739809836464456};
const vector<double> R33 = {0.412327571362291,-0.00888166691871699,0.910992365438493};
const vector<double> T3  = {10.6632863659008,-12.5020142707178,-2.33286297823336};


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
  ros::Publisher cam_obs_pub;
  ros::Publisher pointcloud_pub;

  /** Subscribers **/
  ros::Subscriber cam_obs_sub;
  ros::Subscriber lidar_sub;

  /** Parameters **/
  string base_frame_;
  string camera_frame_;
  string lidar_frame_;
  string map_frame_;

  tf2_ros::TransformBroadcaster br1_;
  tf2_ros::TransformBroadcaster br2_;
  tf2_ros::TransformBroadcaster br3_;
  tf::TransformListener listener_cam;

  /** Flags **/
  bool isBuild_;

  /** Variables **/
  sensor_msgs::PointCloud camera_obs_points_;


  /** Functions **/
  void tfPublish();
  void obstaclePublish();

  void cam_obs_pub_callback(const sensor_msgs::PointCloud::ConstPtr& input) {
    camera_obs_points_.points.clear();
    geometry_msgs::Point32 point;

    tf::StampedTransform transform_cam;
    geometry_msgs::PoseStamped points_cam;
    points_cam.header.frame_id = camera_frame_;
    geometry_msgs::PoseStamped points_base;
    points_base.header.frame_id = base_frame_;
    
    tf::Stamped<tf::Pose> tf_cam;
    tf::Stamped<tf::Pose> tf_base;

    double point_step = 0.05;

    camera_obs_points_.header.frame_id  = input->header.frame_id;

    if(isBuild_) {
      isBuild_ = false;
      for (int i = 0; i < input->points.size() - 2; i+=3) {

        try {
          listener_cam.lookupTransform(base_frame_, camera_frame_,  
                       ros::Time(0), transform_cam);
        } 
        catch (tf::TransformException ex) {
          return;
        }

        if(fabs(input->points[i+1].x-input->points[i+2].x) > 2) continue;
        if(fabs(input->points[i+1].y-input->points[i+2].y) > 2) continue;

        for (double r = input->points[i+1].x; r < input->points[i+2].x; r+=point_step) {
          for (double c = input->points[i+1].y; c < input->points[i+2].y; c+=point_step) {
            points_cam.pose.position.x = r;
            points_cam.pose.position.y = c;
            points_cam.pose.position.z = input->points[i].z;

            tf::poseStampedMsgToTF(points_cam,tf_cam);
            listener_cam.transformPose(base_frame_,ros::Time(0),tf_cam,camera_frame_,tf_base);
            tf::poseStampedTFToMsg(tf_base,points_base);

            point.x = points_base.pose.position.x;
            point.y = points_base.pose.position.y;
            point.z = points_base.pose.position.z;

            camera_obs_points_.points.push_back(point);
          }
        }     
      }
      isBuild_ = true;
    }
  }


  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    sensor_msgs::PointCloud pointcloud_lidar;
    pointcloud_lidar.header.frame_id = lidar_frame_;

    double max_height = 10;
    double min_height = -2;
    double max_range = 5;
    double max_width = 5;
    double min_width = -5;

    for (sensor_msgs::PointCloud2ConstIterator<float>
            iter_x(*input, "x"), iter_y(*input, "y"), iter_z(*input, "z");
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z) { 
      if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z)) continue;
      if (*iter_z > max_height || *iter_z < min_height) continue;
      if (*iter_y > max_width || *iter_y < min_width) continue;
      if (hypot(*iter_x, *iter_y) > max_range) continue;

      geometry_msgs::Point32 point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      
      pointcloud_lidar.points.push_back(point);
    }
    pointcloud_pub.publish(pointcloud_lidar);

  }

  
};


#endif

    /*
    #calib_time: Mon Dec 10 04:13:16 2018
    #description: use R/T/S to transform campoint c into laser coordinates l=R*S*c+T

    R: 
    4.619579111799e-01 1.120235603441e-01 8.797986191318e-01 
    -8.847258557293e-01 -1.123975717332e-02 4.659762097605e-01 
    6.208903689347e-02 -9.936419827013e-01 9.391784554130e-02

    T: 
    2.566721252215e-02 1.607875013048e-01 -3.012084046145e-01

    S: 
    1.000000000000e+00



    roll  tan^-1 (32/33)
    pitch tan^-1 ( -31/sqrt(32^2+33^))
    yaw   tan^-1 (21/11)

    

    double LeMat1[9] = { 
      0.911020853031573, 0.00519415403124045, 0.412327571362292,
      -0.00168139473597834, 0.999959143617621, -0.00888166691871711,
      -0.412356857895401, 0.00739809836464467, 0.910992365438492 };  //B 
    double LeVector1[4] = { -10.6632863659008, 12.5020142707178, 2.33286297823336 ,1000 };


  "RightRotationMatrix": [0.911020853031573,-0.00168139473597833,-0.412356857895401,
  0.00519415403124034,0.999959143617621,0.00739809836464456,
  0.412327571362291,-0.00888166691871699,0.910992365438493],
  "RightTranslationVector": [10.6632863659008,-12.5020142707178,-2.33286297823336],



    */
