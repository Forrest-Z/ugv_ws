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

  ros::Subscriber station_sub;
  ros::Subscriber seq_sub;
  ros::Subscriber action_sub;
  ros::Subscriber index_sub;


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

  ros::Publisher status_pub;
  ros::Publisher seq_pub;
  ros::Publisher action_pub;

  /** ROS Components **/
  tf::TransformListener listener_map_to_base;

  /** Parameters **/
  string robot_id_;

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
  bool isPauseAction_;


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

  vector<StationInfo> station_list_;

  int task_status_last_;
  

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

  bool isMovingBox_;
  bool isWaitingStop_;
  ros::Time moving_box_timer_;
  ros::Time waiting_stop_timer_;;

  string workspace_folder_;
  tf::StampedTransform stampedtransform;


  /** Functions **/
  void ApplyAutoControl(ros::Time& Timer,double& Duration_Limit);
  void ApplyJoyControl();
  void CheckNavigationState(geometry_msgs::Point32 Goal,geometry_msgs::Twist& Cmd_vel);
  void ComputeGlobalPlan(geometry_msgs::Point32& Goal);
  double ComputeMinDistance();
  void ConvertPointcloud(sensor_msgs::PointCloud Input,sensor_msgs::PointCloud& Output,tf::Transform Transform);
  void ConvertPoint(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output,tf::Transform Transform);
  int FindCurrentGoalRoute(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Robot,double Lookahead);
  void LimitCommand(geometry_msgs::Twist& Cmd_vel);
  void PrintConfig();
  bool ReadConfig();
  bool ReadStationList();
  bool UpdateVehicleLocation();
  void UpdateVehicleStatus();


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
    last_timer_ = ros::Time::now();
    static bool isLock = false;
    double axis_deadzone = 0.1;

    (Input->buttons[BUTTON_LB]) ? isManualControl_ = true : isManualControl_ = false;
    (Input->buttons[BUTTON_LB]) ? isJoyControl_ = true : isJoyControl_ = false;

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

  }

  void CmdCallback(const geometry_msgs::Twist::ConstPtr& Input) {
    last_timer_ = ros::Time::now();
    isManualControl_ = true;
    last_timer_ = ros::Time::now();
    geometry_msgs::Twist android_cmd;
    android_cmd.linear.x = Input->linear.x;
    android_cmd.angular.z = Input->angular.z;
    android_cmd.angular.z *= 1.5;
    joy_cmd_ = android_cmd;
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& Input) {
    goal_in_map_.x = Input->pose.position.x;
    goal_in_map_.y = Input->pose.position.y;
    // cout << goal_in_map_.x << "," << goal_in_map_.y << endl;
    ComputeGlobalPlan(goal_in_map_);
    // cout << goal_in_map_.x << "," << goal_in_map_.y << endl << endl;
  }

  void StationCallback(const std_msgs::String::ConstPtr& Input) {
    string require_id = Input->data;

    for (int i = 0; i < station_list_.size(); ++i) {
      if(station_list_[i].id == require_id) {
        goal_in_map_.x = station_list_[i].x;
        goal_in_map_.y = station_list_[i].y;
        ComputeGlobalPlan(goal_in_map_);
        std_msgs::Int32 status;
        int status_data = 1;
        task_status_last_ = status_data;
        status.data = status_data;
        status_pub.publish(status);
        return;
      }
    }
    cout << "Called Station ID : " << require_id << " Not Exist" << endl;
  }


  void SequenceCallback(const std_msgs::String::ConstPtr& Input) {
    string temp_data = Input->data;
    static string last_data; 
    if(temp_data == last_data) return;
    last_data = temp_data; 
    std_msgs::String temp_topic;
    temp_topic.data = temp_data;
    seq_pub.publish(temp_topic);
    // cout << robot_id_ << " Sequence Update To " << temp_data << endl;
  }

  void ActionCallback(const std_msgs::String::ConstPtr& Input) {
    string temp_data = Input->data;
    static string last_data; 
    if(temp_data == last_data) return;
    last_data = temp_data;
    std_msgs::String temp_topic;
    temp_topic.data = temp_data;
    action_pub.publish(temp_topic);
    // cout << robot_id_ << " Current Mission Index " << temp_data << endl;
  }

  void IndexCallback(const std_msgs::Int32::ConstPtr& Input) {
    int temp_data = Input->data;
    std_msgs::Int32 status;
    int status_data = 1;
    status.data = status_data;

    if(temp_data == 1) {
       isPauseAction_ = false;
    } 
    else if(temp_data == 3) {
      isMovingBox_ = true;
      moving_box_timer_ = ros::Time::now();
      status_pub.publish(status);
      task_status_last_ = -1;
    } else {
      isWaitingStop_ = true;
      isPauseAction_ = true;
      waiting_stop_timer_ = ros::Time::now();
      status_pub.publish(status);
      task_status_last_ = -1;
    } 
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
