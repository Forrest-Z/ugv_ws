#ifndef LIB_MISSIONCONTROL_H
#define LIB_MISSIONCONTROL_H

#include <Routing.h>
#include <Planning.h>
#include <Controlling.h>
#include <Tools.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>

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
  bool isManualControl_;
  bool isJoyControl_;
  bool isLocationUpdate_;
  bool isReachCurrentGoal_;


  /** Variables **/
  Tools MyTools_;
  Planning MyPlanner_;
  Controlling MyController_;
  Routing MyRouter_;

  geometry_msgs::Point32 vehicle_in_map_;
  geometry_msgs::Point32 goal_in_map_;

  geometry_msgs::Twist joy_cmd_;

  tf::Transform map_to_base_;
  tf::Transform base_to_map_;

  sensor_msgs::PointCloud global_path_pointcloud_;
  sensor_msgs::PointCloud obstacle_in_base_;


  int map_number_;

  /* 
  uninit   -1
  wait     0
  move     1
  rotation 2
  back     3
  */
  int planner_state_;
  ros::Time last_timer_;

  tf::StampedTransform stampedtransform;


  /** Functions **/
  void ApplyAutoControl(ros::Time& Timer,double& Duration_Limit);
  void ApplyJoyControl();
  void CheckNavigationState(geometry_msgs::Point32 Goal,geometry_msgs::Twist& Cmd_vel);
  void ComputeGlobalPlan(geometry_msgs::Point32& Goal);
  void ConvertPoint(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output,tf::Transform Transform);
  int FindCurrentGoalRoute(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Robot,double Lookahead);
  void LimitCommand(geometry_msgs::Twist& Cmd_vel);
  void PrintConfig();
  bool ReadConfig();
  bool UpdateVehicleLocation();


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
    // cout << goal_in_map_.x << "," << goal_in_map_.y << endl;
    ComputeGlobalPlan(goal_in_map_);
    // cout << goal_in_map_.x << "," << goal_in_map_.y << endl << endl;
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

  
};
#endif
