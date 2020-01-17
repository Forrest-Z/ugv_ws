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

	}
	~Controlling(){};

	void set_local_goal(geometry_msgs::Point32 Input) { local_goal_ = Input; };
	void set_speed_scale(double Input) { speed_scale_ = Input; };
	void set_rotation_scale(double Input) { rotation_scale_ = Input; };

	void ComputePurePursuitCommand(geometry_msgs::Point32 Goal_Route, geometry_msgs::Point32 Goal_Plan,geometry_msgs::Twist& Pp_command);

private:
	const int ROS_RATE_HZ   = 20;
	const double PI = 3.14159265359;

	/** Parameters **/
	double speed_scale_;
	double rotation_scale_;
	/** Variables **/
	geometry_msgs::Point32 local_goal_;


};
#endif