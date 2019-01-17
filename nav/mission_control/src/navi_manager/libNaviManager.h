#ifndef LIB_NAVIMANAGER_H
#define LIB_NAVIMANAGER_H

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
  ros::Publisher vel_pub;
  ros::Publisher move_base_goal_pub;
  ros::Publisher move_base_cancel_pub;
  ros::Publisher call_map_pub;
  ros::Publisher navi_state_pub;
  ros::Publisher odom_pub;
  ros::Publisher pp_pub;


	/** Subscribers **/
	ros::Subscriber plan_sub;
	ros::Subscriber vel_sub;
	ros::Subscriber goal_sub;
	ros::Subscriber map_sub;
	ros::Subscriber obs_sub;
	ros::Subscriber astar_state_sub;
	ros::Subscriber initpose_sub;

	/** Parameters **/
	string junction_file_;

	string base_frame_;
	string camera_frame_;
	string lidar_frame_;
	string map_frame_;

	double speed_scale_;
  double rotation_scale_ ;
	double lookahead_distance_;

	tf::TransformBroadcaster tf_odom;
	tf::TransformListener listener;
	tf::TransformListener listener_local;
	tf::TransformListener listener_obs;
	/** Flags **/
	bool isNewGoal_;
	bool isUseSim_;
	bool isMapSave_;
	bool isJunSave_;
	bool isRecObs_;
	bool isGoalReached_;
	bool isRecovery_;

	/** Variables **/
	nav_msgs::OccupancyGrid static_map_;
	nav_msgs::MapMetaData static_map_info_;
	geometry_msgs::Twist local_cmd_vel_;
	nav_msgs::Path global_path_;

	
	vector<double> robot_position_;
	//vector<vector<double>> waypoint_list_;
	sensor_msgs::PointCloud junction_list_;
	

	std::vector<geometry_msgs::Point32> obs_point_;
	geometry_msgs::Point32 next_goal_;
	geometry_msgs::Point32 navi_goal_;
	/** Functions **/
	void recordLog(string input,LogState level);
	void simDriving(bool flag);
	void publishStaticLayer();
	void loadJunctionFile(string filename);
	void publishJunctionPoints();
	bool followPurePursuit();
	void findPurePursuitGoal();

	void publishCurrentGoal();
	int findPointFromTwoZone(double input_x,double input_y);
	int findPointFromThreeZone(double input_x,double input_y);
	int isReadyToChangeMap(double input_x,double input_y);
	int findJunctionIndex(int goal,int robot);

	void visualPath();



	int findPointZone(double input_x,double input_y);
	geometry_msgs::PoseStamped findJunctionPoint();
	void findRoutingZone();

	inline double sign(double input) {
		return input < 0.0 ? -1.0 : 1.0;
	} 

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
		isGoalReached_ = false;

		if(!isNewGoal_) return;
		//recordLog("New Plan Received",LogState::INFOMATION);
		global_path_ = *input;


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
		vis_pub.publish(marker);

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
		isNewGoal_ = false;

	}

	void vel_callback(const geometry_msgs::Twist::ConstPtr& input) {
		double scale = 0.05;
		double rotation = scale * input->angular.z;
		double speed = scale * input->linear.x;

		robot_position_[2] += rotation;
		robot_position_[2] = remainder(robot_position_[2],2*PI);
		robot_position_[0] += speed * cos(robot_position_[2]);
		robot_position_[1] += speed * sin(robot_position_[2]);
	}

	void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& input) {
		navi_goal_.x = input->pose.position.x;
		navi_goal_.y = input->pose.position.y;
		isGoalReached_ = false;
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
		const int obs_num_max = 20;
		if(obs_point_.size()>20) obs_point_.clear();
		geometry_msgs::Point32 point;
		point.x = input->point.x;
		point.y = input->point.y;
		obs_point_.push_back(point);
	}


	void initpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input) {
		if(!isUseSim_) return;
		robot_position_[0] = input->pose.pose.position.x;
		robot_position_[1] = input->pose.pose.position.y;
		recordLog("Simulation Robot Position Received",LogState::INFOMATION);
		std_msgs::Int32 map_number;
		map_number.data = findPointZone(robot_position_[0],robot_position_[1]);
		isMapSave_ = false;
  	call_map_pub.publish(map_number);
	}


	void astar_state_callback(const std_msgs::Int32::ConstPtr& input) {
		int nav_state = input->data;

		switch(nav_state)
		{
			case 0: {

			break;
			}

			case 1:{
			break;
			}

			case 2:{
				recordLog("Plan Failed",LogState::WARNNING);
				isRecovery_ = true;
			break;
			}

			default: 
				assert(false);
		}
	}




};
#endif