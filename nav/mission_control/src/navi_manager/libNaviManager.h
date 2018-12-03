#ifndef LIB_NAVIMANAGER_H
#define LIB_NAVIMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::to_string;
using std::getline;
using std::ifstream;

const int ROS_RATE_HZ   = 20;
const int ZERO          = 0;
const double PI = 3.14159265359;

const string ORIGIN = "\033[0m";
const string RED    = "\033[91m"; 
const string GREEN  = "\033[92m";
const string YELLOW = "\033[33m"; 
const string WHITE  = "\033[97m";

const int BUTTON_LB             = 4;
const int BUTTON_RB             = 5;

const int AXIS_LEFT_LEFT_RIGHT  = 0;
const int AXIS_LEFT_UP_DOWN     = 1;
const int AXIS_RIGHT_LEFT_RIGHT = 3;
const int AXIS_RIGHT_UP_DOWN    = 4;

;

enum class LogState 
{
	INITIALIZATION,
	INFOMATION,
	STATE_REPORT,
	WARNNING,
	ERROR
}; 

class NaviManager
{
public:
	NaviManager();
	~NaviManager();

	void Manager();
	void paramInitialization();
	void Mission();

private:
	/** Node Handles **/
	ros::NodeHandle n;
	ros::NodeHandle pn;

	/** Publishers **/
	ros::Publisher log_pub;
	ros::Publisher path_pub;
	ros::Publisher waypoint_pub;
  ros::Publisher vis_pub;
  ros::Publisher wall_pub;
  ros::Publisher junction_pub;

	tf::TransformBroadcaster tf_odom;
	tf::TransformListener listener;
	tf::TransformListener listener_local;
	tf::TransformListener listener_obs;

	/** Subscribers **/
	ros::Subscriber plan_sub;
	ros::Subscriber vel_sub;
	ros::Subscriber goal_sub;
	ros::Subscriber map_sub;
	ros::Subscriber obs_sub;

	/** Parameters **/
	string junction_file_;

	/** Flags **/
	bool isNewGoal_;
	bool isGetPath_;
	bool isUseSim_;
	bool isMapSave_;
	bool isJunSave_;
	bool isRecObs_;

	/** Variables **/
	nav_msgs::OccupancyGrid static_map_;
	nav_msgs::MapMetaData static_map_info_;
	
	vector<double> robot_position_;
	//vector<vector<double>> waypoint_list_;
	sensor_msgs::PointCloud junction_list_;

	geometry_msgs::Point32 obs_point_;
	/** Functions **/
	void recordLog(string input,LogState level);
	void simDriving(bool flag);
	void publishStaticLayer();
	void loadJunctionFile(string filename);
	void publishJunctionPoints();

	/** Callbacks **/
	void plan_callback(const nav_msgs::Path::ConstPtr& input) {
		geometry_msgs::Point32 point;
		sensor_msgs::PointCloud pointcloud_path;
		sensor_msgs::PointCloud pointcloud_waypoint;
		pointcloud_path.header = input->header;
		pointcloud_waypoint.header = input->header;

		double assume_acc;
		double assume_spd;
		double max_speed = 2;

		vector<double> speed_box;

		if(input->poses.size() <= 0) return;
		isGetPath_ = true;

		if(!isNewGoal_) return;
		isNewGoal_ = false;

		int unit_step = input->poses.size()/100;
		int startup_points = input->poses.size()/5;
		int lookahead_step = startup_points - unit_step;

		double threshold_curve = 0.05;

		double diff_x;
		double diff_y;
		double per_curve;
		double aft_curve;

		for (int i = 0; i < input->poses.size(); ++i) {
			if (i <= startup_points) {
				assume_acc = max_speed/startup_points;
				assume_spd += assume_acc;
			}
			else if (i > input->poses.size() - startup_points) {
				assume_acc = -max_speed/startup_points;
				assume_spd += assume_acc;
			} else {
				diff_x = input->poses[i+lookahead_step].pose.position.x 
					- input->poses[i+lookahead_step-10*unit_step].pose.position.x;

				diff_y = input->poses[i+lookahead_step].pose.position.y 
					- input->poses[i+lookahead_step-10*unit_step].pose.position.y;

				per_curve = std::atan2(diff_y,diff_x);

				diff_x = input->poses[i+lookahead_step+unit_step].pose.position.x 
					- input->poses[i+lookahead_step].pose.position.x;
				diff_y = input->poses[i+lookahead_step+unit_step].pose.position.y 
					- input->poses[i+lookahead_step].pose.position.y;
				aft_curve = std::atan2(diff_y,diff_x);

				//cout << fabs(aft_curve-per_curve) << endl;	
	
				( fabs(per_curve - aft_curve) < threshold_curve ) ? 
					assume_acc = max_speed/startup_points : 
						assume_acc = -max_speed/startup_points;

				assume_spd += assume_acc;
			}

			if(assume_spd < 0.1) assume_spd = 0.1;
			if(assume_spd > max_speed) assume_spd = max_speed;

			point.x = input->poses[i].pose.position.x;
			point.y = input->poses[i].pose.position.y;
			point.z = assume_spd;
			pointcloud_path.points.push_back(point);
			speed_box.push_back(assume_spd);
		}


		double lookahead_distance = 20;
		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::DELETEALL;
		vis_pub.publish( marker );

		double travel_distance_x = input->poses[0].pose.position.x - input->poses[1].pose.position.x;
		double travel_distance_y = input->poses[0].pose.position.y - input->poses[1].pose.position.y;
		double travel_distance_unit = hypot(travel_distance_x,travel_distance_y); 

		int step = lookahead_distance / travel_distance_unit;

		for (int i = 0; i < input->poses.size(); i+=step) {
			point.x = input->poses[i].pose.position.x;
			point.y = input->poses[i].pose.position.y;
			point.z = speed_box[i];
			pointcloud_waypoint.points.push_back(point);

			// Create an output string stream
			std::ostringstream streamObj3;
			streamObj3 << std::fixed;
			streamObj3 << std::setprecision(2);
			streamObj3 << speed_box[i];

			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "waypoint_speed";
			marker.id = i;
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = point.x + 1;
			marker.pose.position.y = point.y + 1;
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.text = streamObj3.str();//to_string(round(speed_box[i]*100)/100);
			vis_pub.publish( marker );
		}
		waypoint_pub.publish(pointcloud_waypoint);
		path_pub.publish(pointcloud_path);

	}

	void vel_callback(const geometry_msgs::Twist::ConstPtr& input) {
		double scale = 0.1;
		double rotation = scale * input->angular.z;
		double speed = scale * input->linear.x;

		robot_position_[2] += rotation;
		robot_position_[0] += speed * cos(robot_position_[2]);
		robot_position_[1] += speed * sin(robot_position_[2]);
	}

	void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& input) {
		static double last_goal_x = 0;
		static double last_goal_y = 0;

		if (!isGetPath_) return;

		if (input->pose.position.x == last_goal_x && input->pose.position.y == last_goal_y) {
		 	isNewGoal_ = false;
		} else {
			isNewGoal_ = true;
			last_goal_x = input->pose.position.x;
			last_goal_y = input->pose.position.y;
		}	
	}

	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input) {
		if (isMapSave_) return;

		recordLog("Map Received",LogState::INFOMATION);
		recordLog("Map Size " + to_string(input->info.width)
			+ "x" + to_string(input->info.height),LogState::INFOMATION);

		if (input->info.width > 0 && input->info.height > 0) isMapSave_ = true;
		static_map_ = *input;
		static_map_info_ = input->info;
	}

	void obs_callback(const geometry_msgs::PointStamped::ConstPtr& input) {
		recordLog("Simulation Obstacle Received",LogState::INFOMATION);
		recordLog("Point at ( " + to_string(input->point.x)
			+ "," + to_string(input->point.y) + " )",LogState::INFOMATION);
		isRecObs_ = true;

		obs_point_.x = input->point.x;
		obs_point_.y = input->point.y;

	}


};
#endif