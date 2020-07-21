#ifndef _EXECUTING_H_
#define _EXECUTING_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <math.h>


class Executing
{
public:
    Executing();
    ~Executing();

    geometry_msgs::Twist ApplyAutoP(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoR1(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoR2(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoR3(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoN(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoD1(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoD2(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoD3(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoD4(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoAUTO(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoRevolute1(geometry_msgs::Twist Input_cmd);
    geometry_msgs::Twist ApplyAutoRevolute2(geometry_msgs::Twist Input_cmd);
    void ComputeRevoluteCommand(geometry_msgs::Point32 now_pose, geometry_msgs::Point32 start_Pose, double end_theta, geometry_msgs::Twist& Pp_command);
    void SetMaxLinearVel(double Input) {max_linear_velocity_ = Input;}

private:
    const double PI;
    double rotation_scale_;
    double max_linear_velocity_;
};

#endif