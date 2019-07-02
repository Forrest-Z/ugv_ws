#ifndef LIB_CONTROLLING_H
#define LIB_CONTROLLING_H

#include <ros/ros.h>
#include <iostream>
#include <iomanip> 
#include <cmath>
#include <vector>
#include <time.h>
#include <mutex>

#include <boost/make_shared.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/Empty.h>

#include <actionlib_msgs/GoalID.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::isnan;
using std::to_string;
using std::ceil;
using std::setw;


class Controlling
{
public:
	Controlling(){
		vehicle_radius_ = 1;
		inflation_rate_ = 1.5;
		lookahead_time_ = 3;
		oscillation_ = 0.02;

		tried_count_ = 0;
		isTooClose_ = false;
	}
	Controlling(double vehicle_radius,double inflation_rate,double lookahead_time,double oscillation){
		vehicle_radius_ = vehicle_radius;
		inflation_rate_ = inflation_rate;
		lookahead_time_ = lookahead_time;
		oscillation_ = oscillation;

		tried_count_ = 0;
		isTooClose_ = false;
	}
	~Controlling(){};

	void getMovingState(geometry_msgs::Twist Cmd_Vel);
	void getPredictPath(geometry_msgs::Twist Cmd_Vel);
	void filterPointClouds(sensor_msgs::PointCloud Cloud_In,geometry_msgs::Twist Cmd_Vel);
	void computeSafePath(geometry_msgs::Twist& Cmd_Vel);
	bool waitObstaclePass(geometry_msgs::Twist& Cmd_Vel);
	
	sensor_msgs::PointCloud path_predict_;
	sensor_msgs::PointCloud pointcloud_filltered_;
	sensor_msgs::PointCloud pointcloud_onpath_;
	sensor_msgs::PointCloud pointcloud_semicircle_;
	sensor_msgs::PointCloud pointcloud_obstacle_;
	string moving_state_;

private:
	const int ROS_RATE_HZ   = 20;
	const double STOP_TIME_SEC = 0;
	const double PI = 3.14159265359;

	double inflation_rate_;
	double vehicle_radius_;
	double lookahead_time_;
	double oscillation_;
	bool isTooClose_;

	int tried_count_;

	void computeObstacleForce(geometry_msgs::Point32& Force);
	void computeFromSemi(geometry_msgs::Point32& Force,geometry_msgs::Twist& Cmd_Vel);
	void computeClearPath(geometry_msgs::Point32& Force,geometry_msgs::Twist& Cmd_Vel);
	void computeSafeSpeed(geometry_msgs::Point32& Force,geometry_msgs::Twist& Cmd_Vel);
	
	void drawLine(double Begin,double End,double Step,geometry_msgs::Point32 Origin,sensor_msgs::PointCloud& Output) {
		geometry_msgs::Point32 point;
	  if(Begin > End) {
	  	for (double i = Begin; i > End; i-=Step) {
		    point.x = Origin.x + i;
		    point.y = Origin.y;
		    point.z = 0;
		    Output.points.push_back(point);
	  	}
		} else {
			for (double i = Begin; i < End; i+=Step) {
		    point.x = Origin.x + i;
		    point.y = Origin.y;
		    point.z = 0;
		    Output.points.push_back(point);
	  	}
		}
	}

	void drawCircle(double Step,double Radius,sensor_msgs::PointCloud& Output) {
		Step *= PI/(180);
		geometry_msgs::Point32 point;
	  for (double i = 0; i < 2*PI; i+=Step) {
	    point.x = Radius*cos(i);
	    point.y = Radius*sin(i);
	    point.z = 0;
	    Output.points.push_back(point);
	  }
	}

	void drawCircle(double Begin,double End,double Step,double Radius,sensor_msgs::PointCloud& Output) {
		Step *= PI/(180);
		geometry_msgs::Point32 point;
		if(Begin > End) {
			for (double i = Begin; i > End; i-=Step) {
		    point.x = Radius*cos(i);
		    point.y = Radius*sin(i);
		    point.z = 0;
		    Output.points.push_back(point);
	  	}
		} else {
			for (double i = Begin; i < End; i+=Step) {
		    point.x = Radius*cos(i);
		    point.y = Radius*sin(i);
		    point.z = 0;
		    Output.points.push_back(point);
	  	}
		}
	}

	void drawCircle(double Begin,double End,double Step,double Radius,geometry_msgs::Point32 Origin,sensor_msgs::PointCloud& Output) {
		Step *= PI/(180);
		geometry_msgs::Point32 point;
		if(Begin > End) {
			for (double i = Begin; i > End; i-=Step) {
		    point.x = Origin.x + Radius*cos(i);
		    point.y = Origin.y + Radius*sin(i);
		    point.z = 0;
		    Output.points.push_back(point);
	  	}
		} else {
			for (double i = Begin; i < End; i+=Step) {
		    point.x = Origin.x + Radius*cos(i);
		    point.y = Origin.y + Radius*sin(i);
		    point.z = 0;
		    Output.points.push_back(point);
	  	}
		}

	}

};
#endif