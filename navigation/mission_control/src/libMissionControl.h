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

  /** Subscribers **/
  ros::Subscriber joy_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber map_number_sub;
  ros::Subscriber task_sub;
  ros::Subscriber step_sub;
  ros::Subscriber cmd_sub;
  ros::Subscriber face_sub;


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

  ros::Publisher action_pub;

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
  bool isNewCommand_;

  bool isGetFaceID_;


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
  int step_index_;

  tf::StampedTransform stampedtransform;
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
  
  void RunStepCase(int Input);


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
    }

    if(isManualControl_ && Input->buttons[BUTTON_A]) {
      if(step_index_ < 5) {
        global_path_pointcloud_.points.clear();
        step_index_++;
        isReachCurrentGoal_ = true;
        isNewCommand_ = true;
        cout << "Jump to State :" << step_index_ << endl;
        RunStepCase(step_index_);
      }
    }

    if(isManualControl_ && Input->buttons[BUTTON_B]) {
      if(step_index_ > 1) {
        global_path_pointcloud_.points.clear();
        step_index_--;
        isReachCurrentGoal_ = true;
        isNewCommand_ = true;
        cout << "Back to State :" << step_index_ << endl;
        RunStepCase(step_index_);
      }
    }
  }

  void CmdCallback(const geometry_msgs::Twist::ConstPtr& Input) {
    // isManualControl_ = true;
    geometry_msgs::Twist android_cmd;
    android_cmd.linear.x = Input->linear.x;
    android_cmd.angular.z = Input->angular.z;
    android_cmd.angular.z *= 1.5;
    for (int i = 0; i < 2; ++i) {
      cmd_vel_pub.publish(android_cmd);
      ros::Rate loop_rate(ROS_RATE_HZ);
      loop_rate.sleep();
    }
    
    // isManualControl_ = false;
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

  void MapNumberCallback(const std_msgs::Int32::ConstPtr& Input) {
    map_number_ = Input->data;
  }

  void TaskNumberCallback(const std_msgs::Int32MultiArray::ConstPtr& input);

  void StepNumberCallback(const std_msgs::String::ConstPtr& Input) {
    int input_int = stoi(Input->data);
    cout << "Button Input :" << input_int << endl;
    if(input_int == 4) {
      cout << "OPEN CAB" << endl;
      for (int i = 0; i < 20; ++i) {
        geometry_msgs::Twist open_cab;
        open_cab.linear.x = 0;
        open_cab.angular.x = 13;
        open_cab.angular.z = 0;
        cmd_vel_pub.publish(open_cab);
        ros::Rate loop_rate(ROS_RATE_HZ);
        loop_rate.sleep();
      }
    } else {
      if(step_index_ != input_int) {
        step_index_ = input_int;
        isNewCommand_ = true;
      }
      
    }
  } 

  void FaceCallback(const std_msgs::String::ConstPtr& Input){
    if(Input->data == "") return;
    isGetFaceID_ = true;
    int input_int = stoi(Input->data);
    cout << "face id :" << input_int << endl;

  }
  
};
#endif
