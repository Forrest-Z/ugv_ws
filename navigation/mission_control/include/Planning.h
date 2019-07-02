#ifndef LIB_PLANNING_H
#define LIB_PLANNING_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>
#include <limits.h>
#include <algorithm> 

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib_msgs/GoalID.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::to_string;
using std::getline;
using std::ifstream;
using std::min;



class Planning
{
public:
	Planning(double speed_scale,double rotation_scale){
		speed_scale_ = speed_scale;
		rotation_scale_ = rotation_scale;
	};
	Planning(){
		speed_scale_ = 0.3;
		rotation_scale_ = 0.5;
	};
	~Planning(){};

	void findCurrentGoal(nav_msgs::Path global_path,geometry_msgs::Point32 robot_position,geometry_msgs::Point32& current_goal);
	void computePurePursuitCommand(geometry_msgs::Point32 current_goal,geometry_msgs::Point32 robot_position,geometry_msgs::Twist& pp_command);


private:
	const double PI = 3.14159265359;
	double speed_scale_;
	double rotation_scale_;
};
#endif