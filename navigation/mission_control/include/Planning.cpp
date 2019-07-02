#include <Planning.h>

void Planning::findCurrentGoal(nav_msgs::Path global_path,geometry_msgs::Point32 robot_position,geometry_msgs::Point32& current_goal) {
  // Searching for nearest sub-goal use next sub-goal as current goal.
  int min_index;
  double min_distance = DBL_MAX;
  for (int i = 0; i < global_path.poses.size(); i++) {
    double goal_distance_from_robot = 
    hypot((global_path.poses[i].pose.position.x - robot_position.x),
      (global_path.poses[i].pose.position.y - robot_position.y));
    if (goal_distance_from_robot < min_distance) {
      min_distance = goal_distance_from_robot;
      min_index = i;
    }
  }

  // If nearest sub-goal is final goal.
  if(min_index >= global_path.poses.size() - 1) {
    current_goal.x = global_path.poses[global_path.poses.size()-1].pose.position.x;
    current_goal.y = global_path.poses[global_path.poses.size()-1].pose.position.y;
  } else {
    current_goal.x = global_path.poses[min_index+1].pose.position.x;
    current_goal.y = global_path.poses[min_index+1].pose.position.y;
  }

}

void Planning::computePurePursuitCommand(geometry_msgs::Point32 current_goal,geometry_msgs::Point32 robot_position,geometry_msgs::Twist& pp_command) {
  //Using Pure-Presuit Method to compute desire command for current goal.
  double u_turn_speed = 0; 
  double u_turn_rotation = 1.2;
  double rotation_threshold = PI/2;

  double goal_distance = hypot(current_goal.x - robot_position.x,
    current_goal.y - robot_position.y);
  double goal_yaw = atan2(current_goal.y - robot_position.y,
    current_goal.x - robot_position.x);

  double temp_index = 0;
  if (goal_yaw < 0) temp_index = 2;
  if (robot_position.z < 0) temp_index = -2;

  double rotation = goal_yaw  - robot_position.z + temp_index*PI;
  if (rotation < -PI) rotation += 2*PI;
  else if (rotation > PI) rotation -= 2*PI;

  // Sub-goal placed nonuniform, it's hard to set static factor for convert distance to speed, predefined scale only make command smaller.
  if (fabs(rotation) > rotation_threshold) {
    pp_command.angular.z = (rotation/fabs(rotation)) * u_turn_rotation;
    pp_command.linear.x = u_turn_speed;
  } else {
    pp_command.linear.x =  speed_scale_ * goal_distance;
    pp_command.angular.z = rotation_scale_ * rotation;
  }

  // cout<<"rotation "<<rotation*180/PI<<endl;
  // cout<<"goal_distance "<<goal_distance<<endl;
}
