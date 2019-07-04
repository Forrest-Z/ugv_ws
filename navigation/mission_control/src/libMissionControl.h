#ifndef LIB_MISSIONCONTROL_H
#define LIB_MISSIONCONTROL_H

#include <Routing.h>
#include <Planning.h>
#include <Controlling.h>


class MissionControl
{
public:
	MissionControl();
	~MissionControl(); 
  void Initialization();

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


	/** Node Handle **/
	ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Subscribers **/
  ros::Subscriber goal_sub;
  ros::Subscriber map_number_sub;
  ros::Subscriber joy_sub;
  ros::Subscriber map_obs_sub;
  ros::Subscriber odom_sub;

  /** Publishers **/
  ros::Publisher plan_pub;
  ros::Publisher plan_point_pub;
  ros::Publisher vel_pub;
  ros::Publisher path_pred_pub;
  ros::Publisher obs_pub;
  ros::Publisher map_number_pub;
  ros::Publisher clean_path_arrow;

  /** Parameters **/
  double vehicle_radius_;
  double lookahead_time_;

  double max_speed_;
  double max_rotation_;

  double max_acc_speed_;
  double max_acc_rotation_;

  double oscillation_;

  /** Flag **/
  bool isJoy_;
  bool isPatrol_;
  bool isUseSim_;
  bool isJunSave_;

  /** Variables **/
  geometry_msgs::Point32 vehicle_in_map_;
  geometry_msgs::Point32 goal_in_map_;
  int map_number_;
  nav_msgs::Path global_path_;
  geometry_msgs::Twist last_command_;

  sensor_msgs::PointCloud junction_list_;
  geometry_msgs::Point32 current_goal_;

  sensor_msgs::PointCloud map_obs_;

  double joy_speed_;
	double joy_rotation_;
  int sp_cmd_;

  int patrolIndex_;
  vector<geometry_msgs::Point32> patrolA_;
  vector<geometry_msgs::Point32> patrolB_;

  tf::TransformListener listener;



  void makeGlobalPath(geometry_msgs::Point32 goal_in_map);
  void publishPlan(std::vector<MapGraph> Map,std::vector<int> Path,YamlInfo Map_info);

  void pathPredict(geometry_msgs::Twist& Cmd_Vel,sensor_msgs::PointCloud& Cloud_In,sensor_msgs::PointCloud& Cloud_Out);
	void speedLimit(geometry_msgs::Twist& Cmd_Vel);
  void speedSmoother(geometry_msgs::Twist& Cmd_Vel);
  double smootherLogic(double cmd,double last,double acc);
  // bool checkAllStateReady();

  void runPatrolMission(int case_num);

  void checkMapNumber();
  void loadJunctionFile();
  int findPointZone(double input_x,double input_y);
  int isReadyToChangeMap(double input_x,double input_y);
  int findJunctionIndex(int goal,int robot);

  void initPatrol(){
    patrolIndex_ = 0;

    geometry_msgs::Point32 goal;
    goal.x = 44.565;
    goal.y = -34.934;
    patrolA_.push_back(goal);
    goal.x = -88.596;
    goal.y = -33.770;
    patrolA_.push_back(goal);
    goal.x = -76.665;
    goal.y = 50.942;
    patrolA_.push_back(goal);
    goal.x = 2.746;
    goal.y = 92.521;
    patrolA_.push_back(goal);
    patrolB_ = patrolA_;
  }

  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& input) {
    isPatrol_ = false;
  	
    goal_in_map_.x = input->pose.position.x;
    goal_in_map_.y = input->pose.position.y;

    makeGlobalPath(goal_in_map_);
  }

  
  void map_obs_callback(const sensor_msgs::PointCloud::ConstPtr& input) {
    map_obs_ = *input;
  }

  void map_number_callback(const std_msgs::Int32::ConstPtr& input) {
    map_number_ = input->data;
  }

  void joy_callback(const sensor_msgs::Joy::ConstPtr& input) {
  	double joy_x;
	  double joy_y;
	  double min_axis = 0.1;
	  double scale = 1;
    static bool isPower = true;

    if (input->buttons[BUTTON_X] && input->buttons[BUTTON_LB]) isPower = false;
    if (input->buttons[BUTTON_Y] && input->buttons[BUTTON_LB]) isPower = true;

  	(input->buttons[BUTTON_LB]) ? isJoy_ = true : isJoy_ = false;
  	(input->buttons[BUTTON_RB]) ? scale = 0.5 : scale = 1;

  	joy_x = input->axes[AXIS_LEFT_UP_DOWN];
	  joy_y = input->axes[AXIS_RIGHT_LEFT_RIGHT];

	  if (fabs(joy_x)<min_axis) joy_x = 0;
	  if (fabs(joy_y)<min_axis) joy_y = 0;

	  if (isJoy_) {
	    joy_speed_    = scale * joy_x * max_speed_;
	    joy_rotation_ = scale * joy_y * max_rotation_;
	  } else {
	    joy_speed_    = 0;
	    joy_rotation_ = 0;
	  }

    if(!isPower){
      isJoy_ = true;
      joy_speed_    = 0;
      joy_rotation_ = 0;
    }

    if (isJoy_ && input->buttons[BUTTON_BACK]) global_path_.poses.clear();

    geometry_msgs::Point32 position_subway;
    position_subway.x = 2288.519775;
    position_subway.y = 514.973816;

    geometry_msgs::Point32 position_home;
    position_home.x = -23.512510;
    position_home.y = -35.236523;

    // goal_a: [643.462646,-30.777496]
    //   goal_b: [2288.519775,514.973816]
    //   goal_x: [-23.512510,-35.236523]
    //   goal_y: [11.995888,11.043774]

    if (isJoy_ && input->buttons[BUTTON_A]) goal_in_map_ = position_subway;
    if (isJoy_ && input->buttons[BUTTON_B]) goal_in_map_ = position_home;

    if (isJoy_ && input->buttons[BUTTON_B]) goal_in_map_ = position_home;

    sp_cmd_ = 0;
    if (isJoy_ && input->buttons[BUTTON_START]) sp_cmd_=1;

  }


  void odom_callback(const nav_msgs::Odometry::ConstPtr& input) {
    geometry_msgs::TransformStamped odom_trans;

    tf::Transform map_to_odom;
    tf::Transform odom_to_base;
    tf::Transform map_to_base;

    if(isUseSim_){
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/odom";
      odom_trans.child_frame_id = "/base_link";

      odom_trans.transform.translation.x = input->pose.pose.position.x;
      odom_trans.transform.translation.y = input->pose.pose.position.y;
      odom_trans.transform.translation.z = input->pose.pose.position.z;
      odom_trans.transform.rotation = input->pose.pose.orientation;
      transformMsgToTF(odom_trans.transform,odom_to_base);

      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/map";
      odom_trans.child_frame_id = "/odom";

      odom_trans.transform.translation.x = 0;
      odom_trans.transform.translation.y = 0;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,1.57);
      transformMsgToTF(odom_trans.transform,map_to_odom);
    } else {
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/odom";
      odom_trans.child_frame_id = "/base_link";

      odom_trans.transform.translation.x = 0;
      odom_trans.transform.translation.y = 0;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
      transformMsgToTF(odom_trans.transform,odom_to_base);

      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "/map";
      odom_trans.child_frame_id = "/odom";

      odom_trans.transform.translation.x = input->pose.pose.position.x;
      odom_trans.transform.translation.y = input->pose.pose.position.y;
      odom_trans.transform.translation.z = input->pose.pose.position.z;
      odom_trans.transform.rotation = input->pose.pose.orientation;
      transformMsgToTF(odom_trans.transform,map_to_odom);      
    }

    map_to_base = map_to_odom * odom_to_base;
    map_to_base = map_to_base.inverse();

    vehicle_in_map_.x = map_to_base.getOrigin().x();
    vehicle_in_map_.y = map_to_base.getOrigin().y();
    vehicle_in_map_.z = tf::getYaw(map_to_base.getRotation());

    // cout<< vehicle_in_map_.x << ","<<vehicle_in_map_.y << "," << vehicle_in_map_.z << endl;
  }

  inline double sign(double input) {
    return input < 0.0 ? -1.0 : 1.0;
  } 

	
};

#endif
