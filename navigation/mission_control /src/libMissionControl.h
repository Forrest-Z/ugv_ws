#ifndef LIB_MISSIONCONTROL_H
#define LIB_MISSIONCONTROL_H

#include <Routing.h>
#include <Planning.h>
#include <Controlling.h>
#include <Tools.h>
#include <Executing.h>
#include <Supervising.h>
#include <mission_control/AutoSuperviseState.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include "time.h"

using std::to_string;
using std::string;

struct StationInfo {
  string id;
  double x;
  double y;
};

class MissionControl
{
public:
  MissionControl();
  ~MissionControl(); 
  bool Initialization();
  void Execute();

private:
  const int ROS_RATE_HZ = 20;
  const double PI = 3.14159265359;

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

  const int AXIS_CROSS_LEFT_RIGHT = 6;
  const int AXIS_CROSS_UP_DOWN    = 7;

  /** Node Handle **/
  ros::NodeHandle n;
  ros::NodeHandle pn;
  /** Subscribers **/
  ros::Subscriber joy_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber step_sub;
  ros::Subscriber cmd_sub;
  ros::Subscriber tcp_sub;  
  ros::Subscriber map_number_sub;


  /** Publishers **/
  ros::Publisher cmd_vel_pub;
  ros::Publisher path_pred_pub;

  ros::Publisher local_goal_pub;
  ros::Publisher global_goal_pub;

  ros::Publisher local_all_pub;
  ros::Publisher local_safe_pub;
  ros::Publisher local_best_pub;
  ros::Publisher local_costmap_pub;

  ros::Publisher station_points_pub;
  ros::Publisher global_path_pub;
  ros::Publisher vehicle_model_pub;
  ros::Publisher vehicle_info_pub; 

  ros::Publisher traceback_pub;
  ros::Publisher auto_supervise_state_pub;
  ros::Publisher vehicle_run_state_pub;

  /** ROS Components **/
  tf::TransformListener listener_map_to_base;

  /** Parameters **/
  string config_folder_;

  double max_linear_velocity_;
  double max_rotation_velocity_;
  double max_linear_acceleration_;
  double max_rotation_acceleration_;

  double max_translational_velocity_;
  double min_translational_velocity_;

  double vehicle_radius_;
  bool wait_obstacle_;

  double controller_linear_scale_;
  double controller_rotation_scale_;

  /** Flag **/
  bool isWIFIControl_;
  bool isJoyControl_;
  bool isLTEControl_;

  bool isLocationUpdate_;
  bool isReachCurrentGoal_;


  /** Variables **/
  Tools MyTools_;
  Planning MyPlanner_;
  Controlling MyController_;
  Routing MyRouter_;
  Executing MyExecuter_;
  Supervising MySuperviser_;

  geometry_msgs::Point32 vehicle_in_map_;
  geometry_msgs::Point32 goal_in_map_;

  geometry_msgs::Twist joy_cmd_;
  geometry_msgs::Twist wifi_cmd_;
  geometry_msgs::Twist lte_cmd_;

  tf::Transform map_to_base_;
  tf::Transform base_to_map_;

  sensor_msgs::PointCloud global_path_pointcloud_;
  sensor_msgs::PointCloud obstacle_in_base_;

  geometry_msgs::Point32 local_sub_goal_;
  geometry_msgs::Point32 global_sub_goal_;

  std_msgs::String vehicle_run_state_;
  std_msgs::String vehicle_run_state_1st_;
  std_msgs::String vehicle_run_state_2nd_;
  std_msgs::String vehicle_run_state_3rd_;
  std_msgs::String vehicle_run_state_4th_;


  double lookahead_global_meter_;
  double lookahead_local_scale_;


  int map_number_;
  ros::Time last_timer_;

  tf::StampedTransform stampedtransform;

  int planner_decision_;

  bool plan_state_;
  bool wait_plan_state_;

  /** Functions **/
  void ApplyAutoControl(ros::Time& Timer,double& Duration_Limit);

  bool CheckNavigationState();
  void ComputeGlobalPlan(geometry_msgs::Point32& Goal);
  void ConvertPoint(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output,tf::Transform Transform);
  int FindCurrentGoalRoute(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Robot,double Lookahead);
  void LimitCommand(geometry_msgs::Twist& Cmd_vel);
  void PrintConfig();
  bool ReadConfig();
  bool UpdateVehicleLocation();

  void ApplyJoyControl();
  void ApplyWIFIControl();
  void ApplyLTEControl();
  void ApplyStopControl();
  void ApplyBackwardControl();

  void ApplyAutoControl(int mission_state);
  void ApplySlowControl();


  int DecisionMaker();
  int SuperviseDecision(int mission_state);
  geometry_msgs::Twist ExecuteDecision(geometry_msgs::Twist Input,int mission_state,int supervise_state);
  geometry_msgs::Twist TracebackRoute(vector<geometry_msgs::Point32> Input);
  bool CheckTracebackState(geometry_msgs::Point32 Input);

  geometry_msgs::Twist getJoyCommand() {
    return joy_cmd_;
  }
  geometry_msgs::Twist getWIFICommand() {
    return wifi_cmd_;
  }
  geometry_msgs::Twist getLTECommand() {
    return lte_cmd_;
  }

  bool setAutoCoefficient(double index) {
    int search_grid = 3;
    double grid_res = 0.2;
    MyPlanner_.set_safe_path_search_grid(search_grid);
    MyPlanner_.set_costmap_resolution(grid_res);

    MyPlanner_.set_Astar_map_window_radius(Astar_local_map_window_radius_);
	  MyPlanner_.set_RRT_iteration(RRT_iteration_);
	  MyPlanner_.set_RRT_exten_step(RRT_extend_step_);
	  MyPlanner_.set_RRT_search_plan_num(RRT_search_plan_num_);
	  MyPlanner_.set_RRT_expand_size(RRT_expand_size_);
    
    MyController_.set_speed_scale(index * controller_linear_scale_);
    MyController_.set_rotation_scale(index * controller_rotation_scale_);

    lookahead_global_meter_ = 5;
    lookahead_local_scale_ = 2;
    return true;
  }


  geometry_msgs::Twist getAutoCommand();



  void checkCommandSafety(geometry_msgs::Twist raw,geometry_msgs::Twist& safe) {
    safe = raw;
    LimitCommand(safe);
  }
  void createCommandInfo(geometry_msgs::Twist input) {
    MyTools_.BuildPredictPath(input);
  }
  void publishCommand() {
    cmd_vel_pub.publish(MyTools_.cmd_vel());
    path_pred_pub.publish(MyTools_.path_predict());
  }

  void publishInfo() {
    geometry_msgs::Point32 local_goal_map;
    ConvertPoint(local_sub_goal_,local_goal_map,base_to_map_);

    sensor_msgs::PointCloud temp_point;
    temp_point.header.frame_id = "/map";
    temp_point.header.stamp = ros::Time::now();
    local_goal_map.z = 10;
    temp_point.points.push_back(local_goal_map);
    local_goal_pub.publish(temp_point);

    temp_point.points.clear();
    global_sub_goal_.z = 10;
    temp_point.points.push_back(global_sub_goal_);
    global_goal_pub.publish(temp_point);

    if(MyPlanner_.path_all_set().size() > 0) local_all_pub.publish(MyTools_.ConvertVectortoPointcloud(MyPlanner_.path_all_set()));
    if(MyPlanner_.path_safe_set().size() > 0) local_safe_pub.publish(MyTools_.ConvertVectortoPointcloud(MyPlanner_.path_safe_set()));
    local_best_pub.publish(MyPlanner_.path_best());
    global_path_pub.publish(global_path_pointcloud_);

    traceback_pub.publish(MySuperviser_.GetTracebackRoute());

    int* auto_area_state = MySuperviser_.GetAutoAreaState();
    mission_control::AutoSuperviseState auto_supervise_state;
    auto_supervise_state.header.stamp = ros::Time::now();
    auto_supervise_state.danger = auto_area_state[0];
    auto_supervise_state.danger_assist = auto_area_state[1];
    auto_supervise_state.route_outside = auto_area_state[2];
    auto_supervise_state.predict_outside = auto_area_state[3];
    auto_supervise_state.buffer = auto_area_state[4];
    auto_supervise_state.front_safe = auto_area_state[5];
    auto_supervise_state.revolute_safe = auto_area_state[6];
    auto_supervise_state_pub.publish(auto_supervise_state);

    vehicle_run_state_4th_.data = "step4: " + string("danger was ") + to_string(auto_area_state[0]) + 
                                  string(", danger_assist was ") + to_string(auto_area_state[1]) + 
                                  string(", route_outside was ") + to_string(auto_area_state[2]) + 
                                  string(", predict_outside was ") + to_string(auto_area_state[3]) +
                                  string(", buffer was ") + to_string(auto_area_state[4]) +
                                  string(", front_safe was ") + to_string(auto_area_state[5]) +
                                  string(", revolute_safe was ") + to_string(auto_area_state[6]);
    static string temp_run_state_4th;
    if(temp_run_state_4th != vehicle_run_state_4th_.data) {
      std_msgs::String temp_run_state;
      temp_run_state.data = GetLocalTime() + vehicle_run_state_4th_.data;
      vehicle_run_state_pub.publish(temp_run_state);
      temp_run_state_4th = vehicle_run_state_4th_.data;
    }


    RRT_pointcloud_global_.header.frame_id = "/map";
    RRT_pointcloud_global_.header.stamp = ros::Time::now();
    RRT_pointcloud_pub.publish(RRT_pointcloud_global_);
    Astar_pointcloud_global_.header.frame_id = "/map";
    Astar_pointcloud_global_.header.stamp = ros::Time::now();
    Astar_pointcloud_pub.publish(Astar_pointcloud_global_);



  }

  void readVehicleState() {
    static string temp_run_state_1st;
    if(temp_run_state_1st != vehicle_run_state_1st_.data) {
      std_msgs::String temp_run_state;
      temp_run_state.data = GetLocalTime() + vehicle_run_state_1st_.data;
      vehicle_run_state_pub.publish(temp_run_state);
      temp_run_state_1st = vehicle_run_state_1st_.data;
    }

    static string temp_run_state_2nd;
    if(vehicle_run_state_1st_.data == "step1: mission_state was NARROW.") {
      if(temp_run_state_2nd != vehicle_run_state_2nd_.data) {
        std_msgs::String temp_run_state;
        temp_run_state.data = GetLocalTime() + vehicle_run_state_2nd_.data;
        vehicle_run_state_pub.publish(temp_run_state);
        temp_run_state_2nd = vehicle_run_state_2nd_.data;
      }
    }
  }

  bool checkMissionStage() {
    if(global_path_pointcloud_.points.size() > 0) return true;
    return false;
  }

  bool updateState() {
    if(!UpdateVehicleLocation()) {
      cout << "Update Vehicle Location Failed" << endl;
      return false;
    }
    if(!MyPlanner_.UpdateCostmap(obstacle_in_base_)) {
      cout << "Update Costmap Failed" << endl;
      return false;
    }
    if(!MyPlanner_.UpdateAstarCostmap(obstacle_in_base_)) {
      cout << "Update Astar Costmap Failed" << endl;
      return false;
    }
    local_costmap_pub.publish(MyPlanner_.costmap_local());
    if(!CheckNavigationState()) {
      lookahead_Astar_scale_ = 0.9;
      lookahead_RRT_scale_ = 0.9;
      return false;
    }
    return true;
  }



  /** Inline Function **/ 
  inline void ZeroCommand(geometry_msgs::Twist& Cmd_vel) {
    Cmd_vel.linear.x  = 0;
    Cmd_vel.angular.z = 0;
  }

  inline double ComputeSign(double Input){
    return (Input / fabs(Input)); 
  } 

  inline double ComputeMinDistance() {
    double restrict_range = 0.3;
    double obstacle_distance = DBL_MAX;
    double min_distance = DBL_MAX;
    for (int i = 0; i < obstacle_in_base_.points.size(); ++i) {
      obstacle_distance = hypot(obstacle_in_base_.points[i].x,obstacle_in_base_.points[i].y);
      if(obstacle_distance < min_distance) min_distance = obstacle_distance;
    }
    return min_distance;
  }

  /** Callbacks **/
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& Input);
  void CmdCallback(const geometry_msgs::Twist::ConstPtr& Input);
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& Input) {
    goal_in_map_.x = Input->pose.position.x;
    goal_in_map_.y = Input->pose.position.y;
    ComputeGlobalPlan(goal_in_map_);
    narrow_command_stage_ = 0;
  }

  void ObstacleCallback(const sensor_msgs::PointCloud::ConstPtr& Input) {
    obstacle_in_base_ = *Input;
  }

  void MapNumberCallback(const std_msgs::Int32::ConstPtr& Input) {
    map_number_ = Input->data;
  }

  void StepNumberCallback(const std_msgs::String::ConstPtr& Input) {
    int input_int = stoi(Input->data);
  } 

  void TcpCallback(const std_msgs::String::ConstPtr& Input); 

  //========== 增加Astar 与 RRT =======================

  void ApplyNarrowControl(int mission_state);
  geometry_msgs::Twist getNarrowCommand();
  sensor_msgs::PointCloud NarrowAstarPathfind();
  sensor_msgs::PointCloud NarrowRRTPathfind();
  geometry_msgs::Twist PursuitAstarPathCommand();
  geometry_msgs::Twist PursuitRRTPathCommand();
  bool CheckAstarRouteState(geometry_msgs::Point32 Input);
  bool CheckRRTRouteState(geometry_msgs::Point32 Input);
  bool CheckRouteEndState(geometry_msgs::Point32 Input);
  double ComputePathDistance(sensor_msgs::PointCloud Path);
  bool NarrowSubPlannerDecision();
  void CheckNarrowToAutoState();

  int narrow_command_stage_;
  bool isAstarPathfind_;
  bool RRT_refind_;
  double lookahead_Astar_scale_;
  double lookahead_RRT_scale_;
  sensor_msgs::PointCloud Astar_pointcloud_global_;
  sensor_msgs::PointCloud RRT_pointcloud_global_;

  double Astar_local_map_window_radius_;
  double Astar_local_costmap_resolution_;
  int RRT_iteration_;
  double RRT_extend_step_;
  int RRT_search_plan_num_;
	int RRT_expand_size_;

  ros::Publisher RRT_pointcloud_pub;
  ros::Publisher Astar_pointcloud_pub;

  string GetLocalTime() {
    std::time_t tmp_ptr;
    std::tm *tmp_local_ptr = NULL;
    std::time(&tmp_ptr);
    tmp_local_ptr = localtime(&tmp_ptr);

    string time_local = to_string(1900+tmp_local_ptr->tm_year) + string("-") + to_string(1+tmp_local_ptr->tm_mon)
                      + string("-") + to_string(tmp_local_ptr->tm_mday) + string("-") + to_string(tmp_local_ptr->tm_hour)
                      + string("-") + to_string(tmp_local_ptr->tm_min) + string("-") + to_string(tmp_local_ptr->tm_sec) + string(", ");
    return time_local;
  }

  
};
#endif
