#ifndef LIB_RVIZMANAGER_H
#define LIB_RVIZMANAGER_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> //Pose Estimate
#include <geometry_msgs/PoseStamped.h> //Nav Goal
#include <geometry_msgs/PointStamped.h> // Publish Point
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>


#include <std_msgs/Int32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <random>

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

const int ROS_RATE_HZ = 100;
const double PI = 3.14159265359;

struct VehicleMsgs {
  string frame_id;
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
};

class RvizManager
{
public:
  RvizManager();
  ~RvizManager();

  void Manager();
  void Mission();
  void Initialization();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Subscribers & Publishers **/
  ros::Subscriber cmd_sub;
  ros::Subscriber pose_sub;
  
  ros::Subscriber reset_sub;


  /** Parameters **/
  string robot_id_;
  double initX_;
  double initY_;
  double initYaw_;

  tf::TransformBroadcaster tf_base_;

  VehicleMsgs vehicle_info_;
  tf::Transform base_tf_;
  geometry_msgs::Twist input_command_;


  /** Functions **/
  void PublishTF(VehicleMsgs& Input_Vehicle);
  void UpdatePoseSim(geometry_msgs::Twist Input_Cmd, VehicleMsgs& Input_Vehicle);

  void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
  void CmdCallback(const geometry_msgs::Twist::ConstPtr& input) {
    input_command_.linear.x = input->linear.x;
    input_command_.angular.z = input->angular.z;
    UpdatePoseSim(*input,vehicle_info_);
  }
  void ResetCallback(const std_msgs::Int32::ConstPtr& Input) {
    if(Input->data == 0) Initialization();
  }


};


#endif
