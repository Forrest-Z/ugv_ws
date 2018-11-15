#ifndef LIB_SPEEDMANAGER_H
#define LIB_SPEEDMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalID.h>


const std::string ORIGIN = "\033[0m";
const std::string RED    = "\033[91m"; 
const std::string GREEN  = "\033[92m";
const std::string YELLOW = "\033[33m"; 
const std::string WHITE  = "\033[97m";

const int BUTTON_LB             = 4;
const int BUTTON_RB             = 5;

const int AXIS_LEFT_LEFT_RIGHT  = 0;
const int AXIS_LEFT_UP_DOWN     = 1;
const int AXIS_RIGHT_LEFT_RIGHT = 3;
const int AXIS_RIGHT_UP_DOWN    = 4;


const int ROS_RATE_HZ   = 20;
const int STOP_TIME_SEC = 3;
const int ZERO          = 0;
const double PI = 3.14159265359;


enum class ControlState 
{
	JOY_CONTROL,
	ANDROID_CONTROL,
	NAV_CONTROL
}; 

enum class LogState 
{
	INITIALIZATION,
	INFOMATION,
	STATE_REPORT,
	WARNNING,
	ERROR
}; 

using namespace std;

class SpeedManager
{
public:
	SpeedManager();
	~SpeedManager();

	void Manager();
	void paramInitialization();
	void Mission();

private:
	ros::NodeHandle n;
	ros::NodeHandle pn;
	/** Publisher **/
	ros::Publisher vel_pub;
	ros::Publisher pointcloud_pub;
	ros::Publisher visualization_pub;
	ros::Publisher visualization_pub_2;
	ros::Publisher log_pub;
	ros::Publisher move_base_goal_pub;
	ros::Publisher move_base_cancel_pub;

	/** Subscriber **/
	ros::Subscriber joy_sub;
	ros::Subscriber cmd_sub;
	ros::Subscriber scan_sub;
	ros::Subscriber lidar_sub;
	ros::Subscriber android_sub;
	ros::Subscriber nav_state_sub;
	ros::Subscriber button_sub;
	
	/** Variables **/
	double max_speed_;
	double max_rotation_;

	double joy_speed_;
	double joy_rotation_;

	double nav_speed_;
	double nav_rotation_;

	double android_speed_;
	double android_rotation_;

	bool isJoy_;
	bool isNav_;
	bool isAndroid_;
	bool isSupJoy_;


	int lost_signal_ct_;

	std::vector<int> log_schedule_ = {0,0,0,0,0};;
	string last_log_;

	double safe_zone_;
	double lookahead_time_;
	double vehicle_radius_;

	double force_constant_x_;
	double force_constant_y_;

	geometry_msgs::Twist cmd_vel_;
	geometry_msgs::Twist cmd_vel_safe_;

	sensor_msgs::PointCloud collision_points_;

	void drawVehicle(double center_x,double center_y,int seq);
	void drawPath(int seq);
	bool collisionCheck(double input_x, double input_y);
	void collisionAvoid();
	void limitCommand();
	void recordLog(string input,LogState level);
	void debugFunction();

	bool isReceiveJoyCommand();
	bool isReceiveAndroidCommand();
	bool isReceiveNavCommand();

	bool isReceiveCommand(const ControlState input);


	inline double sign(double input)
	{
		return input < 0.0 ? -1.0 : 1.0;
	}

	inline void zeroVelocity()
	{
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    cmd_vel_safe_.linear.x = 0.0;
    cmd_vel_safe_.angular.z = 0.0;
  }


	void joy_callback(const sensor_msgs::Joy::ConstPtr& input)
	{
		// ROS_INFO_ONCE("Joy Message Received");
	  double joy_x;
	  double joy_y;
	  double min_axis = 0.1;

	  (input->buttons[BUTTON_LB]) ? isJoy_ = true : isJoy_ = false;
	  (input->buttons[BUTTON_LB] && input->buttons[BUTTON_RB]) ? isSupJoy_ = true : isSupJoy_ = false;

	  joy_x = input->axes[AXIS_LEFT_UP_DOWN];
	  joy_y = input->axes[AXIS_RIGHT_LEFT_RIGHT];

	  if(fabs(joy_x)<min_axis) joy_x = 0;
	  if(fabs(joy_y)<min_axis) joy_y = 0;

	  if(isJoy_)
	  {
	    joy_speed_    = joy_x * max_speed_;
	    joy_rotation_ = joy_y * max_rotation_;
	  } else {
	    joy_speed_    = 0;
	    joy_rotation_ = 0;
	    // ROS_INFO("JOY CLEAR CMD");
	  }
	}


	void cmd_callback(const geometry_msgs::Twist::ConstPtr& input)
	{
		nav_speed_    = input->linear.x;
	  nav_rotation_ = input->angular.z;
	  isNav_ = true;
	}

	void android_callback(const geometry_msgs::Twist::ConstPtr& input)
	{
	  android_speed_    = input->linear.x;
	  android_rotation_ = input->angular.z;

	  if( android_speed_ == 0 && android_rotation_ == 0) isAndroid_ = false;
	  else isAndroid_ = true;

	}


	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& input)
	{
	  geometry_msgs::Point32 point;
	  sensor_msgs::PointCloud pointcloud_scan;

	  pointcloud_scan.header = input->header;

	  collision_points_.points.clear();

	  for (int i = 0; i < input->ranges.size(); i++){
	    double angle = input->angle_min + i * input->angle_increment;
	    point.x = input->ranges[i] * cos(angle);
	    point.y = input->ranges[i] * sin(angle);

	    if (!collisionCheck(point.x,point.y)) continue;

	    collision_points_.points.push_back(point);
	    pointcloud_scan.points.push_back(point);
	  }

	  pointcloud_pub.publish(pointcloud_scan);		
	}


	void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
	{
		sensor_msgs::PointCloud pointcloud_lidar;
	  pointcloud_lidar.header = input->header;

	  for (sensor_msgs::PointCloud2ConstIterator<float>
	          iter_x(*input, "x"), iter_y(*input, "y"), iter_z(*input, "z");
	          iter_x != iter_x.end();
	          ++iter_x, ++iter_y, ++iter_z)
	  { 
	    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;

	    if (!collisionCheck(*iter_x,*iter_y)) continue;

	    geometry_msgs::Point32 point;
	    point.x = *iter_x;
	    point.y = *iter_y;
	    point.z = *iter_z;
	    
	    pointcloud_lidar.points.push_back(point);
	  }
	  //pointcloud_pub.publish(pointcloud_lidar);
	}


	void button_callback(const std_msgs::String::ConstPtr& input)
	{
		string msg = input->data;
		if(msg == "1")
		{
			double goal_x = 5;
			double goal_y = 5;

			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = "map";
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = goal_x;
      msg.pose.position.y = goal_y; 
      msg.pose.orientation.w =1.0;
      //move_base_goal_pub.publish(msg);

			recordLog("Button 1 | Goal at (" + std::to_string(int(goal_x)) 
				+ "," + std::to_string(int(goal_y))+ ")",LogState::INFOMATION);
		}
		else if(msg == "2")
		{
			double goal_x = 10;
			double goal_y = 10;
			
			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = "map";
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = goal_x;
      msg.pose.position.y = goal_y; 
      msg.pose.orientation.w =1.0;
      //move_base_goal_pub.publish(msg);

			recordLog("Button 2 | Goal at (" + std::to_string(int(goal_x)) 
				+ "," + std::to_string(int(goal_y))+ ")",LogState::INFOMATION);
		}
		else if(msg == "3")
		{
			recordLog("Button 3",LogState::INFOMATION);
		}
		else if(msg == "4")
		{
			recordLog("Button 4",LogState::INFOMATION);
		} else {
			recordLog("Unregister Button",LogState::INFOMATION);
		}

	}


	void nav_state_callback(const std_msgs::Int32::ConstPtr& input)
	{
		int nav_state = input->data;

		switch(nav_state)
		{
			case 0:
			{

			}
			break;

			case 1:
			{
				isNav_ = false;
			}
			break;

			case 2:
			{
				isAndroid_ = false;
			}

			default: 
			break;
		}
	}

};
#endif