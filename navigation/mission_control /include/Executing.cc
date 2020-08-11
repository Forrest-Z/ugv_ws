#include "Executing.h"

using std::cout;
using std::endl;

Executing::Executing():PI(3.14159265){
    rotation_scale_ = 4;
    max_linear_velocity_ = 3;
    cout << "Executing is running" << endl;
}
Executing::~Executing(){
    cout << "Executing is closed" << endl;
}

geometry_msgs::Twist Executing::ApplyAutoP(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = 0;
    raw_cmd.angular.z = 0;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoR1(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = -1;
    raw_cmd.angular.z = 0.1;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoR2(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = -1;
    raw_cmd.angular.z = -0.1;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoR3(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = -1;
    raw_cmd.angular.z = 0;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoN(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = 0;
    raw_cmd.angular.z = 0;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoD1(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = 0.5;
    raw_cmd.angular.z = 0;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoD2(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = max_linear_velocity_/4;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoD3(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = max_linear_velocity_/2;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoD4(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = max_linear_velocity_/1.5;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoAUTO(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = Input_cmd.linear.x;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoRevolute1(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = Input_cmd.linear.x;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}
geometry_msgs::Twist Executing::ApplyAutoRevolute2(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = Input_cmd.linear.x;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}
void Executing::ComputeRevoluteCommand(geometry_msgs::Point32 now_pose, geometry_msgs::Point32 start_pose, double end_theta, geometry_msgs::Twist& Pp_command) {
    double yaw_now = now_pose.z;
    if(end_theta < 0){
        if(yaw_now - start_pose.z <= 0) yaw_now = yaw_now;
        else if(yaw_now - start_pose.z > 0) yaw_now = yaw_now - 2*PI;
    }else if(end_theta > 0){
        if(yaw_now - start_pose.z >= 0) yaw_now = yaw_now;
        else if(yaw_now - start_pose.z < 0) yaw_now = yaw_now + 2*PI;
    }

    double goal_angular_distance = end_theta - (yaw_now - start_pose.z); 
    Pp_command.linear.x =  -0.01;
    Pp_command.angular.z = rotation_scale_ * goal_angular_distance;

}

//============== narrow 规划器 ==========================
geometry_msgs::Twist Executing::ApplyNarrowP(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = 0;
    raw_cmd.angular.z = 0;
    return raw_cmd;
}

geometry_msgs::Twist Executing::ApplyNarrowD1(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    if(Input_cmd.linear.x > 1) Input_cmd.linear.x = 2;
    raw_cmd.linear.x = Input_cmd.linear.x;
    raw_cmd.angular.z = Input_cmd.angular.z;
    return raw_cmd;
}

geometry_msgs::Twist Executing::ApplyNarrowR1(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = 0;
    raw_cmd.angular.z = 1;
    return raw_cmd;
}

geometry_msgs::Twist Executing::ApplyNarrowR2(geometry_msgs::Twist Input_cmd){
    geometry_msgs::Twist raw_cmd;
    raw_cmd.linear.x = 0;
    raw_cmd.angular.z = -1;
    return raw_cmd;
}

