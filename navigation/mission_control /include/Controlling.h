#ifndef LIB_CONTROLLING_H
#define LIB_CONTROLLING_H

#include <ros/ros.h>
#include <iostream>
#include <iomanip> 
#include <cmath>
#include <vector>
#include <time.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

class Controlling
{
public:
	Controlling(){
		speed_scale_ = 2;
		rotation_scale_ = 2;
		max_linear_acceleration_ = 0.015;

	}
	~Controlling(){};
	void set_speed_scale(double Input) { speed_scale_ = Input; };
	void set_rotation_scale(double Input) { rotation_scale_ = Input; };
	void setMaxLinearAcceleration(double Input) { temp_max_linear_acceleration_ = Input; }
	void getMaxLinearAcceleration(double &Output) { Output = max_linear_acceleration_; }

	void ComputePurePursuitCommand(geometry_msgs::Point32 Goal_Route, geometry_msgs::Point32 Goal_Plan,geometry_msgs::Twist& Pp_command);
	void ComputePurePursuitRRTCommand(geometry_msgs::Point32 Goal_Route, geometry_msgs::Point32 Goal_Plan,geometry_msgs::Twist& Pp_command);

private:
	const int ROS_RATE_HZ   = 20;
	const double PI = 3.14159265359;

	/** Parameters **/
	double speed_scale_;
	double rotation_scale_;
	double max_linear_acceleration_;
	double temp_max_linear_acceleration_;
};
#endif