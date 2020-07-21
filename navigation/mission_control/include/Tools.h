#ifndef LIB_TOOLS_H
#define LIB_TOOLS_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;

class Tools
{
public:
  Tools() {
  	vehicle_radius_ = 0.3;
		lookahead_time_ = 3;
		oscillation_    = 0.05;
  }
  Tools(double vehicle_radius,double lookahead_time,double oscillation) {
  	vehicle_radius_ = vehicle_radius;
		lookahead_time_ = lookahead_time;
		oscillation_    = oscillation;
  }
  ~Tools(){}

  void BuildPredictPath(geometry_msgs::Twist Cmd_vel);

  sensor_msgs::PointCloud ConvertVectortoPointcloud(vector<sensor_msgs::PointCloud> Input);
  sensor_msgs::PointCloud path_predict() { return (path_predict_); }
  geometry_msgs::Twist cmd_vel() { return (cmd_vel_); }

private: 
	const double PI = 3.14159265359;
	
	/** Parameters **/
	double vehicle_radius_;
	double lookahead_time_;
	double oscillation_;

	/** Variables **/
	sensor_msgs::PointCloud path_predict_;
	string moving_state_;
	geometry_msgs::Twist cmd_vel_;

	/** Functions **/
	void GetMovingState(geometry_msgs::Twist Cmd_Vel);
	void DrawLine(double Begin,double End,double Step,geometry_msgs::Point32 Origin,sensor_msgs::PointCloud& Output);

	void DrawCircle(double Step,double Radius,sensor_msgs::PointCloud& Output);
	void DrawCircle(double Begin,double End,double Step,double Radius,sensor_msgs::PointCloud& Output);
  void DrawCircle(double Begin,double End,double Step,double Radius,geometry_msgs::Point32 Origin,sensor_msgs::PointCloud& Output);


};


#endif
