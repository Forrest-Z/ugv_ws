#ifndef LIB_MISSIONCONTROL_H
#define LIB_MISSIONCONTROL_H

#include <Routing.h>
#include <Planning.h>
#include <Controlling.h>
#include <Tools.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>

class MissionControl
{
public:
  MissionControl();
  ~MissionControl(); 
  bool Initialization();
  void Execute();

private:
  const int ROS_RATE_HZ = 50;
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

  /** Subscribers **/
  ros::Subscriber joy_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber map_number_sub;
  ros::Subscriber task_sub;

  /** Publishers **/
  ros::Publisher path_pred_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher local_costmap_pub;

  ros::Publisher local_goal_pub;
  ros::Publisher local_all_pub;
  ros::Publisher local_safe_pub;
  ros::Publisher local_best_pub;

  ros::Publisher global_goal_pub;
  ros::Publisher global_path_pub;

  ros::Publisher map_number_pub;
  ros::Publisher junction_points_pub;
  ros::Publisher station_points_pub;
  ros::Publisher vehicle_model_pub; 

  /** ROS Components **/
  tf::TransformListener listener_map_to_base;

  /** Parameters **/
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
  bool isLocationUpdate_;
  bool isJunSave_;
  bool isReachCurrentGoal_;
  bool isTaskFinished_;

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
  sensor_msgs::PointCloud junction_list_;

  sensor_msgs::PointCloud task_list_;
  sensor_msgs::PointCloud current_task_;
  sensor_msgs::PointCloud task_log_;

  int map_number_;

  int task_index_;
  /* 
  uninit   -1
  wait     0
  move     1
  rotation 2
  back     3
  */
  int planner_state_;

  string workspace_folder_;

  /** Functions **/
  void ApplyAutoControl(ros::Time& Timer,double& Duration_Limit);
  void ApplyJoyControl();
  void CheckNavigationState(geometry_msgs::Point32 Goal,geometry_msgs::Twist& Cmd_vel);
  double ComputeMinDistance();
  void CheckMapNumber(); 
  void ComputeGlobalPlan(geometry_msgs::Point32& Goal);
  int ComputeJunctionDistance(geometry_msgs::Point32 Input);
  void ConvertPointcloud(sensor_msgs::PointCloud Input,sensor_msgs::PointCloud& Output,tf::Transform Transform);
  void ConvertPoint(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output,tf::Transform Transform);
  int FindCurrentGoalRoute(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Robot,double Lookahead);
  void LimitCommand(geometry_msgs::Twist& Cmd_vel);
  bool LoadJunctionFile();
  void PrintConfig();
  bool ReadConfig();
  bool UpdateVehicleLocation();

  bool ReadTaskList();
  void UpdateTask();
  

  /** Inline Function **/ 
  inline void ZeroCommand(geometry_msgs::Twist& Cmd_vel) {
    Cmd_vel.linear.x  = 0;
    Cmd_vel.angular.z = 0;
  }

  inline double ComputeSign(double Input){
    return (Input / fabs(Input)); 
  } 

  inline string GetCurrentWorkingDir() {
    string cwd("\0",FILENAME_MAX+1);
    return getcwd(&cwd[0],cwd.capacity());
  }



  /** Callbacks **/
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& Input) {
    static bool isLock = false;
    double axis_deadzone = 0.1;

    (Input->buttons[BUTTON_LB]) ? isManualControl_ = true : isManualControl_ = false;

    if (Input->buttons[BUTTON_LB] && Input->buttons[BUTTON_X]) isLock = true;
    if (Input->buttons[BUTTON_LB] && Input->buttons[BUTTON_Y]) isLock = false;

    double axis_x = Input->axes[AXIS_LEFT_UP_DOWN];
    double axis_y = Input->axes[AXIS_RIGHT_LEFT_RIGHT];
    if(fabs(axis_x) < axis_deadzone) axis_x = 0;
    if(fabs(axis_y) < axis_deadzone) axis_y = 0;

    if(isLock) {
      isManualControl_ = true;
      ZeroCommand(joy_cmd_);
    } else {
      joy_cmd_.linear.x  = axis_x * max_linear_velocity_;
      joy_cmd_.angular.z = axis_y * max_rotation_velocity_;      
    }

    if(isManualControl_ && Input->buttons[BUTTON_BACK]) {
      global_path_pointcloud_.points.clear();
      current_task_.points.clear();
      task_index_ = 1;
    }
    if(isManualControl_ && Input->buttons[BUTTON_A]) {
      if(isReachCurrentGoal_) {
        if(current_task_.points.size() < 1) {
          cout << "Finished All Tasks" << endl;
          global_path_pointcloud_.points.clear();
          current_task_.points.clear();
          task_log_.points.clear();
          task_index_ = 1;
        } else {
          current_task_.points.erase (current_task_.points.begin());
          global_path_pointcloud_.points.clear();
          goal_in_map_ = current_task_.points.front();
          cout << "Station Confirm move to next Station " << current_task_.points.front().z << endl;
          ComputeGlobalPlan(goal_in_map_);
        }
      } else {
        if(current_task_.points.size() > 0){
          goal_in_map_ = current_task_.points.front();
          ComputeGlobalPlan(goal_in_map_);
        }
      }
    }

    if(Input->axes[AXIS_CROSS_UP_DOWN] > 0) {
      cout << "Current Task Show" << endl;
      for (int i = 0; i < task_log_.points.size(); ++i) {
        cout << "Task Index : " << task_log_.points[i].x 
        << " | Station ID : " << task_log_.points[i].y << endl;
      }
    }

  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input) {
    goal_in_map_.x = input->pose.position.x;
    goal_in_map_.y = input->pose.position.y;
    // cout << goal_in_map_.x << "," << goal_in_map_.y << endl;
    ComputeGlobalPlan(goal_in_map_);
    // cout << goal_in_map_.x << "," << goal_in_map_.y << endl << endl;
  }

  void ObstacleCallback(const sensor_msgs::PointCloud::ConstPtr& Input) {
    obstacle_in_base_ = *Input;
  }

  void MapNumberCallback(const std_msgs::Int32::ConstPtr& input) {
    map_number_ = input->data;
  }

  void TaskNumberCallback(const std_msgs::Int32MultiArray::ConstPtr& input);
  
};
#endif
