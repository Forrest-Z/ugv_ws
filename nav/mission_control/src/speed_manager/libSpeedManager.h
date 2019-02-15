#ifndef LIB_SPEEDMANAGER_H
#define LIB_SPEEDMANAGER_H

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

using cv::Mat;
using cv::Vec3b;

const string ORIGIN = "\033[0m";
const string RED    = "\033[91m"; 
const string GREEN  = "\033[92m";
const string YELLOW = "\033[33m"; 
const string WHITE  = "\033[97m";

const int BUTTON_LB             = 4;
const int BUTTON_RB             = 5;

const int BUTTON_A              = 0;
const int BUTTON_B              = 1;
const int BUTTON_X              = 2;
const int BUTTON_Y              = 3;

const int BUTTON_BACK           = 6;
const int BUTTON_START          = 7;

const int AXIS_LEFT_LEFT_RIGHT  = 0;
const int AXIS_LEFT_UP_DOWN     = 1;
const int AXIS_RIGHT_LEFT_RIGHT = 3;
const int AXIS_RIGHT_UP_DOWN    = 4;


const int ROS_RATE_HZ   = 20;
const int STOP_TIME_SEC = 3;
const int ZERO          = 0;
const double PI = 3.14159265359;

enum class LogState 
{
	INITIALIZATION,
	INFOMATION,
	STATE_REPORT,
	WARNNING,
	ERROR
}; 

struct Distance2D {
	double left;
	double right;
};

struct LaneFrame {
	Distance2D distance;
	double direction;
};



class SpeedManager
{
public:
	SpeedManager();
	~SpeedManager();

	void Manager();
	void paramInitialization();
	void Mission();


private:
	/** Node Handles **/
	ros::NodeHandle n;
	ros::NodeHandle pn;

	/** Publishers **/
	ros::Publisher vel_pub;
	ros::Publisher pointcloud_pub;
	ros::Publisher visualization_pub;
	ros::Publisher visualization_pub_2;
	ros::Publisher log_pub;
	ros::Publisher goal_pub;
	ros::Publisher lane_pub; 


	/** Subscribers **/
	ros::Subscriber joy_sub;
	ros::Subscriber cmd_sub;
	ros::Subscriber local_cmd_sub;
	ros::Subscriber lidar_sub;
	ros::Subscriber android_sub;
	ros::Subscriber nav_state_sub;
	ros::Subscriber button_sub;
	ros::Subscriber wall_sub;
	ros::Subscriber power_sub;

	/** Services **/
	ros::ServiceClient clear_costmap_client;
	

	/** Parameters **/
	double max_speed_;
	double max_rotation_;

	double safe_zone_;
	double vehicle_radius_;
	double lookahead_time_;
	double radius_scale_;

	double force_constant_x_;
	double force_constant_y_;

	double max_height_;
	double min_height_;
	double max_range_;
	double min_range_;

	double oscillation_;
	double play_speed_;

	string base_frame_;
	string camera_frame_;
	string lidar_frame_;
	string map_frame_;

	vector<double> goal_a_;
	vector<double> goal_b_;
	vector<double> goal_x_;
	vector<double> goal_y_;

	/** Flags **/
	int lost_signal_ct_;

	bool isJoy_;
	bool isNav_;
	bool isAndroid_;
	bool isSupJoy_;
	bool isOneHand_;
	bool isRecovery_;

  std::mutex mutex_draw_;
	/** Variables **/
	double joy_speed_;
	double joy_rotation_;

	double nav_speed_;
	double nav_rotation_;

	double android_speed_;
	double android_rotation_;

	geometry_msgs::Twist cmd_vel_;
	geometry_msgs::Twist cmd_vel_safe_;

	sensor_msgs::PointCloud collision_lidar_points_;
	sensor_msgs::PointCloud collision_wall_points_;
	sensor_msgs::PointCloud collision_wall_points_all_;
	sensor_msgs::PointCloud collision_power_points_;

	tf::TransformListener listener;

	/** Functions **/
	void drawVehicle(double center_x,double center_y,int seq);
	void drawPath(int seq);
	bool collisionCheck(double input_x, double input_y);
	void computeForce(sensor_msgs::PointCloud& points
		,double& force_x,double& force_y,double& freespace,bool& flag);
	void collisionAvoid();
	void limitCommand(geometry_msgs::Twist& input_cmd);
	void debugFunction();
	void clearCostmap();
	void recordLog(string input,LogState level);
	void superCommand();
	void recoveryAcion();
	void speedSmoother(geometry_msgs::Twist& input_cmd);
	bool isReceiveCommand();

	int findVehicleState(std::vector<sensor_msgs::PointCloud> Input_Points);
	void publishPathParticle(std::vector<sensor_msgs::PointCloud> Input_Points,
		cv::Mat& Output_Grid);
	void findClearPath(cv::Mat& Input_Grid,geometry_msgs::Twist& Output_Cmd);
	void searchPath(cv::Mat& Input_Grid,
		int Start_Row,int Start_Col,int End_Row,int End_Col);
	void shiftLine(cv::Mat& Input_Grid,geometry_msgs::Twist& Output_Cmd,int& Ref_Timer);

	bool generateLane(sensor_msgs::PointCloud Input,LaneFrame& Output);

	void findAvailableLane(LaneFrame Input,std::vector<sensor_msgs::PointCloud> Input_Points);
	int findObstacleLane(LaneFrame Input, std::vector<sensor_msgs::PointCloud> Input_Points, int Lane_Num, int Vehicle_Lane, int Scale);


	int makeDecision(std::vector<sensor_msgs::PointCloud> Input_Points);

	void filterPointCloud(std::vector<sensor_msgs::PointCloud>& Input_Points);


	inline double sign(double input) {
		return input < 0.0 ? -1.0 : 1.0;
	}

	inline int sign(int input) {
		return input < 0 ? -1 : 1;
	}

	inline void zeroVelocity() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    cmd_vel_safe_.linear.x = 0.0;
    cmd_vel_safe_.angular.z = 0.0;
  }

	/** Callbacks **/

	void joy_callback(const sensor_msgs::Joy::ConstPtr& input) {
		// ROS_INFO_ONCE("Joy Message Received");
	  double joy_x;
	  double joy_y;
	  double min_axis = 0.1;

	  static bool poweron = true;

	  (input->buttons[BUTTON_LB]) ? isJoy_ = true : isJoy_ = false;
	  (input->buttons[BUTTON_LB] && input->buttons[BUTTON_RB]) 
	  	? isSupJoy_ = true : isSupJoy_ = false;

	  if (input->buttons[BUTTON_BACK] && input->buttons[BUTTON_LB]) poweron = false;
	  if (input->buttons[BUTTON_START] && input->buttons[BUTTON_LB]) poweron = true;

	  if (isOneHand_) {
	  	joy_x = input->axes[AXIS_LEFT_UP_DOWN];
	  	joy_y = input->axes[AXIS_LEFT_LEFT_RIGHT];
	  } else {
	  	joy_x = input->axes[AXIS_LEFT_UP_DOWN];
	  	joy_y = input->axes[AXIS_RIGHT_LEFT_RIGHT];
	  }


	  if (fabs(joy_x)<min_axis) joy_x = 0;
	  if (fabs(joy_y)<min_axis) joy_y = 0;

	  if (isJoy_) {
	    joy_speed_    = joy_x * max_speed_;
	    joy_rotation_ = joy_y * max_rotation_;
	  } else {
	    joy_speed_    = 0;
	    joy_rotation_ = 0;
	    // ROS_INFO("JOY CLEAR CMD");
	  }

	  if (!poweron) {
	  	isJoy_ = true;
	  	joy_speed_    = 0;
	    joy_rotation_ = 0;
	    recordLog("Power off",LogState::WARNNING);
	  }


	  boost::shared_ptr<std_msgs::String> sim_button(new std_msgs::String());

	  if (input->buttons[BUTTON_A] && input->buttons[BUTTON_LB]) {
	  	sim_button->data = "1";
	  	button_callback(sim_button);
	  }
	  if (input->buttons[BUTTON_B] && input->buttons[BUTTON_LB]) {
	  	sim_button->data = "2";
	  	button_callback(sim_button);
	  }
	  if (input->buttons[BUTTON_X] && input->buttons[BUTTON_LB]) {
	  	sim_button->data = "3";
	  	button_callback(sim_button);
	  }
	  if (input->buttons[BUTTON_Y] && input->buttons[BUTTON_LB]) {
	  	sim_button->data = "4";
	  	button_callback(sim_button);
	  }

	}


	void cmd_callback(const geometry_msgs::Twist::ConstPtr& input) {
		// nav_speed_    = input->linear.x;
	 //  nav_rotation_ = input->angular.z;
	 //  isNav_ = true;
	 //  isRecovery_ = false;
	}

	void local_cmd_callback(const geometry_msgs::Twist::ConstPtr& input) {
		nav_speed_    = input->linear.x;
	  nav_rotation_ = input->angular.z;
	  isNav_ = true;
	  isRecovery_ = false;
	}

	void android_callback(const geometry_msgs::Twist::ConstPtr& input) {
	  android_speed_    = input->linear.x;
	  android_rotation_ = input->angular.z;

	  if (android_speed_ == 0 && android_rotation_ == 0) isAndroid_ = false;
	  else isAndroid_ = true;

	}

	void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		tf::StampedTransform transform;
		try {
			listener.lookupTransform(base_frame_, lidar_frame_, ros::Time(0), transform);
		} catch (tf::TransformException ex) {
	    recordLog("Waiting for Lidar to Base",LogState::WARNNING);
	    return;
		}

		pcl::PointCloud<pcl::PointXYZ> cloud_in;
		pcl::PointCloud<pcl::PointXYZ> cloud_trans;
		cloud_trans.header.frame_id = base_frame_;
		sensor_msgs::PointCloud2 cloud_out; 

		pcl::fromROSMsg(*msg,cloud_in);

		pcl_ros::transformPointCloud (cloud_in,cloud_trans,transform);  
		pcl::toROSMsg(cloud_trans,cloud_out);


		//boost::shared_ptr<sensor_msgs::PointCloud2> input(new sensor_msgs::PointCloud2());

		auto input = boost::make_shared<sensor_msgs::PointCloud2>(cloud_out);


		sensor_msgs::PointCloud pointcloud_lidar;
	  pointcloud_lidar.header.frame_id = base_frame_;

	  collision_lidar_points_.points.clear();
	  
	  sensor_msgs::LaserScan output;
	  output.header = pointcloud_lidar.header;
	  output.angle_min = -PI;
    output.angle_max = PI;
    output.angle_increment = 0.00330;
    output.range_max = max_range_;
    output.range_min = min_range_;

    uint32_t ranges_size = ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1);

	  for (sensor_msgs::PointCloud2ConstIterator<float>
	          iter_x(*input, "x"), iter_y(*input, "y"), iter_z(*input, "z");
	          iter_x != iter_x.end();
	          ++iter_x, ++iter_y, ++iter_z) { 
	    if (isnan(*iter_x) || isnan(*iter_y) || isnan(*iter_z)) continue;
	    if (*iter_z > max_height_ || *iter_z < min_height_) continue;

	    double angle = atan2(*iter_y, *iter_x);
	    double range = hypot(*iter_x, *iter_y);
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index]) {
        output.ranges[index] = range;
      }    

	    if (hypot(*iter_x,*iter_y) > 2 * safe_zone_) continue;
	    if (!collisionCheck(*iter_x,*iter_y)) continue;

	    geometry_msgs::Point32 point;
	    point.x = *iter_x;
	    point.y = *iter_y;
	    point.z = *iter_z;
	    
	    pointcloud_lidar.points.push_back(point);
	    point.z = 0;
	    collision_lidar_points_.points.push_back(point);
	  }
	  //pointcloud_pub.publish(pointcloud_lidar);
	  //scan_pub.publish(output);  
	}


	void button_callback(const std_msgs::String::ConstPtr& input) {
		string msg = input->data;
		if (msg == "1") {
			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = goal_a_[0];
      msg.pose.position.y = goal_a_[1]; 
      msg.pose.orientation.w =1.0;
      goal_pub.publish(msg);

			recordLog("Button A | Goal at (" + to_string(int(msg.pose.position.x)) 
				+ "," + to_string(int(msg.pose.position.y))+ ")",LogState::INFOMATION);
		}
		else if (msg == "2") {		
			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = goal_b_[0];
      msg.pose.position.y = goal_b_[1]; 
      msg.pose.orientation.w =1.0;
      goal_pub.publish(msg);

			recordLog("Button B | Goal at (" + to_string(int(msg.pose.position.x)) 
				+ "," + to_string(int(msg.pose.position.y))+ ")",LogState::INFOMATION);
		}
		else if (msg == "3") {
			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = goal_x_[0];
      msg.pose.position.y = goal_x_[1]; 
      msg.pose.orientation.w =1.0;
      goal_pub.publish(msg);

			recordLog("Button X | Goal at (" + to_string(int(msg.pose.position.x)) 
				+ "," + to_string(int(msg.pose.position.y))+ ")",LogState::INFOMATION);
		}
		else if(msg == "4") {
			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = map_frame_;
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = goal_y_[0];
      msg.pose.position.y = goal_y_[1]; 
      msg.pose.orientation.w =1.0;
      goal_pub.publish(msg);

			recordLog("Button Y | Goal at (" + to_string(int(msg.pose.position.x)) 
				+ "," + to_string(int(msg.pose.position.y))+ ")",LogState::INFOMATION);
		} else {
			recordLog("Unregister Button",LogState::INFOMATION);
		}
	}


	void nav_state_callback(const std_msgs::Int32::ConstPtr& input) {
		int nav_state = input->data;

		switch(nav_state)
		{
			case 0: {

			break;
			}

			case 1:{
				isNav_ = false;
				recordLog("Goal Reached",LogState::STATE_REPORT);
			break;
			}

			case 2:{
				
			break;
			}

			default: 
				assert(false);
		}
	}

	void wall_callback(const sensor_msgs::PointCloud::ConstPtr& input) {
		collision_wall_points_.points.clear();
		collision_wall_points_all_.points.clear();

		geometry_msgs::Point32 point;
		for (int i = 0; i < input->points.size(); ++i) {
			point.x = input->points[i].x;
			point.y = input->points[i].y;
			collision_wall_points_all_.points.push_back(point);

			if(!collisionCheck(input->points[i].x,input->points[i].y)) continue;

			collision_wall_points_.points.push_back(point);
		}

		collision_wall_points_.header.frame_id = base_frame_;
		pointcloud_pub.publish(collision_wall_points_);
	}

	void power_callback(const sensor_msgs::PointCloud::ConstPtr& input) {
		collision_power_points_.points.clear();
		geometry_msgs::Point32 point;

		for (int i = 0; i < input->points.size(); ++i) {
			if(!collisionCheck(input->points[i].x,input->points[i].y)) continue;

			point.x = input->points[i].x;
			point.y = input->points[i].y;

			collision_power_points_.points.push_back(point);
		}
	}

};
#endif