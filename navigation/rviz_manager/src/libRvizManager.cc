#include "libRvizManager.h"

RvizManager::RvizManager():pn("~") {
  n.param<string>("robot_id", robot_id_, "");
  n.param<double>("initX", initX_, 0);
  n.param<double>("initY", initY_, 0);
  n.param<double>("initYaw", initYaw_, 0);
  
  cmd_sub = n.subscribe("husky_velocity_controller/cmd_vel",1, &RvizManager::CmdCallback,this);
  pose_sub = n.subscribe("initialpose",1, &RvizManager::PoseCallback,this);

  reset_sub = n.subscribe("/reset_control",1,&RvizManager::ResetCallback,this);

  Initialization();
}

RvizManager::~RvizManager() {
}

void RvizManager::Initialization() {
  vehicle_info_.position.x = initX_;
  vehicle_info_.position.y = initY_;
  vehicle_info_.orientation = tf::createQuaternionMsgFromYaw(initYaw_);
  cout << "RvizManager Initialized " << robot_id_ << " X Y Yaw " << initX_ << " " << initY_ << " " << initYaw_ << endl;
}

void RvizManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  while (ros::ok()) {
  	Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void RvizManager::Mission() {
  PublishTF(vehicle_info_);
}

void RvizManager::PublishTF(VehicleMsgs& Input_Vehicle) {
  geometry_msgs::TransformStamped odom_trans;
  tf::Transform odom_tf;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "/odom";
  odom_trans.child_frame_id = "/base_link";

  odom_trans.transform.translation.x = Input_Vehicle.position.x;
  odom_trans.transform.translation.y = Input_Vehicle.position.y;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = Input_Vehicle.orientation;

  transformMsgToTF(odom_trans.transform,odom_tf);

  base_tf_ = odom_tf;

  tf_base_.sendTransform(odom_trans);
}

void RvizManager::UpdatePoseSim(geometry_msgs::Twist Input_Cmd, VehicleMsgs& Input_Vehicle) {
  double scale_speed = 0.05;
  double scale_rotation = 0.03;
  double speed = scale_speed * Input_Cmd.linear.x;
  double rotation = scale_rotation * Input_Cmd.angular.z;
  double vehicle_yaw = tf::getYaw(Input_Vehicle.orientation);

  static geometry_msgs::Point last_pose = Input_Vehicle.position;
  static ros::Time last_timer = ros::Time::now();
  static double last_yaw = vehicle_yaw;

  Input_Vehicle.position.x += speed * cos(vehicle_yaw);
  Input_Vehicle.position.y += speed * sin(vehicle_yaw);

  double move_x = fabs(last_pose.x - Input_Vehicle.position.x);
  double move_y = fabs(last_pose.y - Input_Vehicle.position.y);
  double move_distance = hypot(move_x,move_y);
  double move_time = (ros::Time::now()- last_timer).toSec();
  double move_speed = move_distance/move_time;
  double rotation_speed = (vehicle_yaw-last_yaw)/move_time;

  // cout << "Simulation Speed :" << move_speed << "m/s | " << rotation_speed << "rad/s" << endl;

  last_timer = ros::Time::now();
  last_pose = Input_Vehicle.position;
  last_yaw = vehicle_yaw;

  vehicle_yaw += rotation;
  Input_Vehicle.orientation = tf::createQuaternionMsgFromYaw(vehicle_yaw);
}

void RvizManager::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input) {
  VehicleMsgs msg_pose;
  tf::Transform init_tf;
  geometry_msgs::Twist init_cmd;

  msg_pose.frame_id = input->header.frame_id;
  msg_pose.position = input->pose.pose.position;
  msg_pose.orientation = input->pose.pose.orientation;
  vehicle_info_ = msg_pose;
  base_tf_ = init_tf;
}

