#ifndef _SUPERVISING_H_
#define _SUPERVISING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <math.h>


enum class AutoState
{
  JOY = 0,
  LTE,
  WIFI,
  AUTO,
  SLOW,
  STOP,
  BACK,
  NARROW,
  ACTION,
};

enum class AUTO
{
    P = 0,
    R1,
    R2,
    R3,
    N,
    D1,
    D2,
    D3,
    D4,
    AUTO,
    REVOLUTE1,
    REVOLUTE2,
    TRACEBACK,  
};

enum class NARROW
{
    P = 0,
    D1,
    R1,
    R2,
};

using std::vector;

class Supervising
{
public:
    
    Supervising();
    ~Supervising();

    void AutoObstaclePercept(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Twist Input_cmd);
    void GenerateTracebackRoute(geometry_msgs::Point32 global_goal, geometry_msgs::Point32 vehicle_pose_now);
    int TracebackSupervise(vector<geometry_msgs::Point32> Input, geometry_msgs::Point32 vehicle_in_map);
    int AutoSuperviseDecision(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Point32 vehicle_in_map,geometry_msgs::Twist Input_cmd,bool plan_state,bool wait_plan_state);
    int AstarSuperviseDecision(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Point32 vehicle_in_map,geometry_msgs::Twist Input_cmd,bool plan_state,bool wait_plan_state);
    geometry_msgs::Point32 GetRevolutePose() {return revolute_pose_;}
    vector<geometry_msgs::Point32> GetVehicleTraceback() {return vehicle_pose_traceback;}
    sensor_msgs::PointCloud GetTracebackRoute() {return traceback_route;}
    int* GetAutoAreaState() {return auto_area_state_;}

    void NarrowObstaclePercept(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Twist Input_cmd);
    int NarrowSuperviseDecision(sensor_msgs::PointCloud obstacle_in_base,geometry_msgs::Twist Input_cmd);
    bool NarrowPlannerRestart(sensor_msgs::PointCloud obstacle_in_base,vector<geometry_msgs::Point32> path_local,nav_msgs::OccupancyGrid Grid,geometry_msgs::Twist Input_cmd);
    bool GetAutoMissionState() {return AUTO_mission_state_;}
    void SetAutoMissionState(bool Input) {AUTO_mission_state_ = Input;}
    bool GetNarrowRefind() {return narrow_refind_;}



private:
    geometry_msgs::Point32 revolute_pose_;
    vector<geometry_msgs::Point32> vehicle_pose_que;         //相邻两个路径点之间的实时路径
    vector<geometry_msgs::Point32> vehicle_pose_traceback;
    sensor_msgs::PointCloud traceback_route;
    int auto_area_state_[7]; 

    const double PI = 3.14159265;
    const int ROS_RATE = 20;

    double Ka;             // 危险区半径比例因子
    double Kb;             // 区域角度比例因子

    //=================区域障碍物状态===================
    bool danger;
    bool route_outside;
    bool route_map;
    bool predict_outside;
    bool predict_map;
    bool buffer;
    bool back_safe_left;
    bool back_safe_right;
    bool front_safe;
    bool revolute_safe;
    bool danger_assist;

    //=================区域半径========================
    double limit_danger_radius;
    double limit_route_radius;
    double limit_predict_radius;
    double buffer_radius;
    double back_safe_radius;
    double front_safe_radius;
    double revolute_safe_radius;
    double danger_assist_radius;
    double min_danger_longth;
    double min_danger_width;

    int back_turn;
    int attempt_limit;
    int attempt_times;
    int traceback_index;
    double angle_offset_;

    inline void ConvertAnglePositive(float& angle, geometry_msgs::Point32 vehicle_in_map) {
        if(vehicle_in_map.z + angle_offset_ > PI) angle = vehicle_in_map.z + angle_offset_ - 2*PI;
        else angle = vehicle_in_map.z + angle_offset_;
    }

    inline void ConvertAngleNegative(float& angle, geometry_msgs::Point32 vehicle_in_map) {
        if(vehicle_in_map.z + angle_offset_ < -PI) angle = vehicle_in_map.z - angle_offset_ + 2*PI;
        else angle = vehicle_in_map.z - angle_offset_;
    }


    bool narrow_danger;
    bool narrow_buffer;
    bool AUTO_mission_state_;
    bool narrow_refind_;
    
    double narrow_danger_longth;
    double narrow_danger_width;
    double narrow_buffer_radius;

    int local_turn;
};

#endif 