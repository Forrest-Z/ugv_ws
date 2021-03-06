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
#include <unistd.h>
#include <sensor_msgs/LaserScan.h>

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
  ros::Subscriber control_sub;  
  ros::Subscriber map_number_sub;

  ros::Subscriber reset_sub;
  ros::Subscriber action_sub;
  ros::Subscriber planner_manual_sub;

  ros::Subscriber scan_sub;
  ros::Subscriber remote_control_sub;

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

  ros::Publisher action_state_pub;
  ros::Publisher plan_str_pub;

  ros::Publisher gate_command_pub;

  /** ROS Components **/
  tf::TransformListener listener_map_to_base;

  /** Parameters **/
  string config_folder_;

  double max_linear_velocity_;
  double max_rotation_velocity_;
  double max_linear_acceleration_;
  double max_rotation_acceleration_;
  double temp_max_linear_acceleration_;

  double max_translational_velocity_;
  double min_translational_velocity_;

  double vehicle_radius_;
  bool wait_obstacle_;

  double controller_linear_scale_;
  double controller_rotation_scale_;

  double limit_danger_radius_;
  double min_danger_longth_;
  double min_danger_width_;
  double limit_route_radius_;
  double limit_predict_radius_;
  double buffer_radius_;
  double back_safe_radius_;
  double front_safe_radius_;
  double revolute_safe_radius_;
  double danger_assist_radius_;
  double spline_search_grid_;

  double spline_costmap_resolution_;
  double RRT_costmap_resolution_;
  double spline_map_window_radius_;
  double RRT_map_window_radius_;
  int spline_expand_size_;

  /** Flag **/
  bool isWIFIControl_;
  bool isJoyControl_;
  bool isLTEControl_;
  bool isLTELock_;
  bool isAction_;

  bool isLocationUpdate_;
  bool isReachCurrentGoal_;
  bool isAdjustGuardPose_;


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

  std_msgs::String action_msg_;

  double lookahead_global_meter_;
  double lookahead_local_scale_;


  int map_number_;
  ros::Time last_timer_;
  ros::Time lte_last_timer_;

  tf::StampedTransform stampedtransform;

  int planner_decision_;

  bool plan_state_;
  bool wait_plan_state_;
  bool planner_manual_state_;
  string planner_manual_;

  bool isJOYscram_;

  ros::Time action_start_timer_;
  int current_action_index_;
  string robot_id_;
  string community_id_;
  int auto_state_global_;


  int lift_mode_;
  int stage_index_;
  int collision_distance_;
  int mirror_num_;
  double linear_speed_,angular_speed_;

  ros::Time beginturn_ = ros::Time::now();

  double k_wall_;
  double y_coordinate_; 
  double angle_right_,angle_left_,h1_,h2_,h3_;
  double elevator_wall_;
  double length_up_,length_down_,length_left_,length_right_;
  double ob_lift_;

  sensor_msgs::PointCloud scan_points_;

  /** Functions **/
  void ApplyAutoControl(ros::Time& Timer,double& Duration_Limit);

  bool CheckNavigationState();
  void ComputeGlobalPlan(geometry_msgs::Point32& Goal);
  void ConvertPoint(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output,tf::Transform Transform);
  int FindCurrentGoalRoute(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Robot,double Lookahead);
  void LimitCommand(geometry_msgs::Twist& Cmd_vel,int mission_state);
  void PrintConfig();
  bool ReadConfig();
  bool UpdateVehicleLocation();

  void ApplyJoyControl(int mission_state);
  void ApplyWIFIControl(int mission_state);
  void ApplyLTEControl(int mission_state);
  void ApplyStopControl(int mission_state);
  void ApplyBackwardControl();

  void ApplyAction(int mission_state);

  void ApplyAutoControl(int mission_state);
  void ApplySlowControl();
  void ApplyNarrowControl(int mission_state);
  void ApplyRelocationControl(int mission_state);
  void ApplyNomapControl(int mission_state);
  void ApplyWaitControl(int mission_state);
  void ApplyRotationControl(int mission_state);
  void ApplyRemoteControl(int mission_state);

  geometry_msgs::Twist getRemoteCommand();


  int DecisionMaker();
  int SuperviseDecision(int mission_state);

  void RemoveMapObstacle(sensor_msgs::PointCloud &obstacle) {
    sensor_msgs::PointCloud temp_obstacle;
    geometry_msgs::Point32 temp_point;
    
    for(int i = 0; i < obstacle.points.size(); i++) {
      if(obstacle.points[i].z != 10) {
        temp_point.x = obstacle.points[i].x;
        temp_point.y = obstacle.points[i].y;
        temp_point.z = obstacle.points[i].z;
        temp_obstacle.points.push_back(temp_point);
      }
    }

    obstacle.points.clear();
    obstacle.points = temp_obstacle.points;
  }

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

  bool setAutoCoefficient(double index,double grid_res,double map_window_radius) {
    MyPlanner_.set_safe_path_search_grid(spline_search_grid_);
    MyPlanner_.set_costmap_resolution(grid_res);
    MyPlanner_.set_map_window_radius(map_window_radius);

    MyPlanner_.set_Astar_map_window_radius(Astar_local_map_window_radius_);
	  MyPlanner_.set_RRT_iteration(RRT_iteration_);
	  MyPlanner_.set_RRT_exten_step(RRT_extend_step_);
	  MyPlanner_.set_RRT_search_plan_num(RRT_search_plan_num_);
	  MyPlanner_.set_RRT_expand_size(RRT_expand_size_);
	  MyPlanner_.set_spline_expand_size(spline_expand_size_);
    
    MyController_.set_speed_scale(index * controller_linear_scale_);
    MyController_.set_rotation_scale(index * controller_rotation_scale_);

    MySuperviser_.set_limit_danger_radius(limit_danger_radius_);
    MySuperviser_.set_min_danger_longth(min_danger_longth_);
    MySuperviser_.set_min_danger_width(min_danger_width_);
    MySuperviser_.set_limit_route_radius(limit_route_radius_);
    MySuperviser_.set_limit_predict_radius(limit_predict_radius_);
    MySuperviser_.set_buffer_radius(buffer_radius_);
    MySuperviser_.set_back_safe_radius(back_safe_radius_);
    MySuperviser_.set_front_safe_radius(front_safe_radius_);
    MySuperviser_.set_revolute_safe_radius(revolute_safe_radius_);
    MySuperviser_.set_danger_assist_radius(danger_assist_radius_);
    
    lookahead_global_meter_ = 5;
    // lookahead_local_scale_ = 3.5;
    lookahead_local_scale_ = 2.5;
    return true;
  }

  geometry_msgs::Twist getAutoCommand();
  bool RelocationDelay();

  int relocation_stage_;

  void checkCommandSafety(geometry_msgs::Twist raw,geometry_msgs::Twist& safe,int mission_state) {
    safe = raw;
    LimitCommand(safe,mission_state);
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

    if(MyPlanner_.path_all_set().size() > 0) local_all_pub.publish(MyPlanner_.ConvertVectortoPointcloud(MyPlanner_.path_all_set()));
    if(MyPlanner_.path_safe_set().size() > 0) local_safe_pub.publish(MyPlanner_.ConvertVectortoPointcloud(MyPlanner_.path_safe_set()));
    local_best_pub.publish(MyPlanner_.sub_1_path_best());
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
    // if(!MyPlanner_.UpdateCostmap(obstacle_in_base_)) {
    //   cout << "Update Costmap Failed" << endl;
    //   return false;
    // }
    
    // local_costmap_pub.publish(MyPlanner_.costmap_local());
    if(!CheckNavigationState()) {
      return false;
    }
    return true;
  }

  bool updateNarrowState() {
    if(!MyPlanner_.UpdateAstarCostmap(obstacle_in_base_)) {
      cout << "Update Astar Costmap Failed" << endl;
      return false;
    }
    return true;
  }

  void SendMqttRoute(vector<int> Input);

  /** Inline Function **/ 
  inline void ZeroCommand(geometry_msgs::Twist& Cmd_vel) {
    Cmd_vel.linear.x  = 0;
    Cmd_vel.angular.z = 0;
  }

  inline void RotationCommand(geometry_msgs::Twist& Cmd_vel) {
    static bool isRotationState = true;
    static double yaw_init;
    if(isRotationState) {
      yaw_init = vehicle_in_map_.z;
      isRotationState = false;
    }

    double yaw_now = vehicle_in_map_.z;
    double rotation_angle = PI/2;
    double angle_offset = 0;

    if(yaw_now - yaw_init + angle_offset < rotation_angle) {
      cout << "yaw_now - yaw_init + angle_offset : " << yaw_now - yaw_init + angle_offset << endl;
      Cmd_vel.linear.x  = 0;
      Cmd_vel.angular.z = 0.3;
    } else {
      Cmd_vel.linear.x  = 0;
      Cmd_vel.angular.z = 0;
      isAdjustGuardPose_ = false;
      isRotationState = true;

      action_msg_.data = "open";
		  gate_command_pub.publish(action_msg_);
    }
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

  inline void ClearAutoMissionState() {
    MySuperviser_.SetAutoMissionState(true);
    ClearNarrowMissionState();
    auto_state_global_ = 0;
  }
  inline void ClearNarrowMissionState() {
    narrow_command_stage_ = 0;
    ClearRRTPlanState();
  }
  inline void ClearRRTPlanState() {
    lookahead_RRT_scale_ = 0.9;
  }

  /** Callbacks **/
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& Input);
  void CmdCallback(const geometry_msgs::Twist::ConstPtr& Input);
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& Input) {
    goal_in_map_.x = Input->pose.position.x;
    goal_in_map_.y = Input->pose.position.y;
    goal_in_map_.z = Input->pose.position.z;
    ComputeGlobalPlan(goal_in_map_);
    ClearAutoMissionState();
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

  void ResetCallback(const std_msgs::Int32::ConstPtr& Input) {
    if(Input->data == 0 || Input->data == 1) Initialization();
  }

  void ControlCallback(const std_msgs::String::ConstPtr& Input); 

  void ActionCallback(const std_msgs::Int32::ConstPtr& Input) {
    if(Input->data == 0) {
      isAction_ = false;
      global_path_pointcloud_.points.clear();
    }
    else if(Input->data == 1) {
      global_path_pointcloud_.points.clear();
      action_start_timer_ = ros::Time::now();
      isAction_ = true;
      current_action_index_ = Input->data;
    } else {
      isAction_ = false;
    }
  }

  void PlannerManualCallback(const std_msgs::String::ConstPtr& Input) {
    if(Input->data != "WAIT" && Input->data != "AUTO" && Input->data != "NARROW" 
      && Input->data != "NOMAP" && Input->data != "RELOCATION" && Input->data != "STOP") {
      planner_manual_state_ = false;
      return;
    } 

    planner_manual_ = Input->data;
    planner_manual_state_ = true;
  }

  void RemoteControlCallback(const std_msgs::String::ConstPtr& Input) {
    int index = Input->data.find("&");
    string head_str = Input->data.substr(0,index);
    string end_str = Input->data.substr(index+1);

    cout << "string: " << head_str << "  " << end_str << endl;

    if(head_str == "1" && end_str == "UP") {
      isRemote_ = true;
      isRemoteFront_ = true; isRemoteBack_ = false; isRemoteLeft_ = false; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = false; isRemoteBackLeft_ = false; isRemoteBackRight_ = false;
    }
    if(head_str == "1" && end_str == "DOWN") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = true; isRemoteLeft_ = false; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = false; isRemoteBackLeft_ = false; isRemoteBackRight_ = false;
    }
    if(head_str == "1" && end_str == "LEFT") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = false; isRemoteLeft_ = true; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = false; isRemoteBackLeft_ = false; isRemoteBackRight_ = false;      
    }
    if(head_str == "1" && end_str == "RIGHT") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = false; isRemoteLeft_ = false; isRemoteRight_ = true; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = false; isRemoteBackLeft_ = false; isRemoteBackRight_ = false;      
    }
    if(head_str == "1" && end_str == "UPLEFT") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = false; isRemoteLeft_ = false; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = true; isRemoteFrontRight_ = false; isRemoteBackLeft_ = false; isRemoteBackRight_ = false;      
    }
    if(head_str == "1" && end_str == "UPRIGHT") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = false; isRemoteLeft_ = false; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = true; isRemoteBackLeft_ = false; isRemoteBackRight_ = false;      
    }
    if(head_str == "1" && end_str == "DOWNLEFT") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = false; isRemoteLeft_ = false; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = false; isRemoteBackLeft_ = true; isRemoteBackRight_ = false;      
    }
    if(head_str == "1" && end_str == "DOWNRIGHT") {
      isRemote_ = true;
      isRemoteFront_ = false; isRemoteBack_ = false; isRemoteLeft_ = false; isRemoteRight_ = false; 
      isRemoteFrontLeft_ = false; isRemoteFrontRight_ = false; isRemoteBackLeft_ = false; isRemoteBackRight_ = true;      
    }

  }


  //========== 增加Astar 与 RRT =======================
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


  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& Input);
  void initLift();
  geometry_msgs::Twist UpLift1Cmd();
  geometry_msgs::Twist DownLift1Cmd();
  geometry_msgs::Twist UpLift17Cmd();
  geometry_msgs::Twist DownLift17Cmd();
  bool GoStraight(geometry_msgs::Twist& Output);
  bool StepBack(geometry_msgs::Twist& Output);
  bool Turn(geometry_msgs::Twist& Output);
  bool EnterLift(geometry_msgs::Twist& Output);
  bool GetOffLift(geometry_msgs::Twist& Output);
  void RansacFrontWall(sensor_msgs::PointCloud Input);
  void DetectObstacles(sensor_msgs::PointCloud Input);
  void Retreat(sensor_msgs::PointCloud Input);
  void DistanceAroundandMirror(sensor_msgs::PointCloud Input);
  void goStraightObstacleDete(sensor_msgs::PointCloud Input);

  bool isRemote_;
  geometry_msgs::Point32 remote_goal_;
  geometry_msgs::Point32 remote_goal_front_;
  geometry_msgs::Point32 remote_goal_back_;
  geometry_msgs::Point32 remote_goal_left_;
  geometry_msgs::Point32 remote_goal_right_;
  geometry_msgs::Point32 remote_goal_frontleft_;
  geometry_msgs::Point32 remote_goal_frontright_;
  geometry_msgs::Point32 remote_goal_backleft_;
  geometry_msgs::Point32 remote_goal_backright_;

  bool isRemoteFront_; 
  bool isRemoteBack_; 
  bool isRemoteLeft_; 
  bool isRemoteRight_; 
  
  bool isRemoteFrontLeft_; 
  bool isRemoteFrontRight_; 
  bool isRemoteBackLeft_; 
  bool isRemoteBackRight_; 

  inline void setRemoteGoal(double Map_radius) {
    // 目标点越接近样条终点,所带来goal的距离差异越大
    remote_goal_front_.x = 2*Map_radius;
    remote_goal_front_.y = 0;

    remote_goal_back_.x = -2*Map_radius;
    remote_goal_back_.y = 0;

    remote_goal_left_.x = 0;
    remote_goal_left_.y = 2*Map_radius;

    remote_goal_right_.x = 0;
    remote_goal_right_.y = -2*Map_radius;

    remote_goal_frontleft_.x = 2*Map_radius;
    remote_goal_frontleft_.y = 2*Map_radius;

    remote_goal_frontright_.x = 2*Map_radius;
    remote_goal_frontright_.y = -2*Map_radius;

    remote_goal_backleft_.x = -2*Map_radius;
    remote_goal_backleft_.y = 2*Map_radius;

    remote_goal_backright_.x = -2*Map_radius;
    remote_goal_backright_.y = -2*Map_radius;

    if(isRemoteFront_) remote_goal_ = remote_goal_front_;
    else if(isRemoteBack_) remote_goal_ = remote_goal_back_;
    else if(isRemoteLeft_) remote_goal_ = remote_goal_left_;
    else if(isRemoteRight_) remote_goal_ = remote_goal_right_;
    else if(isRemoteFrontLeft_) remote_goal_ = remote_goal_frontleft_;
    else if(isRemoteFrontRight_) remote_goal_ = remote_goal_frontright_;
    else if(isRemoteBackLeft_) remote_goal_ = remote_goal_backleft_;
    else if(isRemoteBackRight_) remote_goal_ = remote_goal_backright_;
  }







};
#endif
