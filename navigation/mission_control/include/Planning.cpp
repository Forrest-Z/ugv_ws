#include <Planning.h>

void Planning::findCurrentGoal(nav_msgs::Path global_path,geometry_msgs::Point32 robot_position,geometry_msgs::Point32& current_goal) {
// Searching for nearest sub-goal use next sub-goal as current goal.
  int min_index;
  double min_distance = DBL_MAX;
  static double last_curve = 1;
  double lookahead = 3;

  for (int i = 0; i < global_path.poses.size(); i++) {
    double goal_distance_from_robot = 
    hypot((global_path.poses[i].pose.position.x - robot_position.x),
      (global_path.poses[i].pose.position.y - robot_position.y));
    if (goal_distance_from_robot < min_distance) {
      min_distance = goal_distance_from_robot;
      min_index = i;
    }
  }

  min_distance = DBL_MAX;
  for (int i = min_index; i < global_path.poses.size(); i++) {
    double goal_distance_from_robot = 
    hypot((global_path.poses[i].pose.position.x - robot_position.x),
      (global_path.poses[i].pose.position.y - robot_position.y));
    if (goal_distance_from_robot < lookahead) continue;
    if (goal_distance_from_robot < min_distance) {
      min_distance = goal_distance_from_robot;
      min_index = i;
    }
  }

  if(min_index >= global_path.poses.size() - 1) {
    current_goal.x = global_path.poses[global_path.poses.size()-1].pose.position.x;
    current_goal.y = global_path.poses[global_path.poses.size()-1].pose.position.y;
  } else {
    current_goal.x = global_path.poses[min_index+1].pose.position.x;
    current_goal.y = global_path.poses[min_index+1].pose.position.y;
  }

  if(min_index < global_path.poses.size() - 6) {
    double delta_x12 = global_path.poses[min_index+3].pose.position.x 
                      - global_path.poses[min_index].pose.position.x;
    double delta_x23 = global_path.poses[min_index+5].pose.position.x 
                      - global_path.poses[min_index+3].pose.position.x; 
    double delta_x13 = global_path.poses[min_index+5].pose.position.x 
                      - global_path.poses[min_index].pose.position.x; 
    double delta_y12 = global_path.poses[min_index+3].pose.position.y 
                      - global_path.poses[min_index].pose.position.y;
    double delta_y23 = global_path.poses[min_index+5].pose.position.y 
                      - global_path.poses[min_index+3].pose.position.y;
    double delta_y13 = global_path.poses[min_index+5].pose.position.y 
                      - global_path.poses[min_index].pose.position.y;

    double distance_12 = hypot(delta_x12,delta_y12);
    double distance_23 = hypot(delta_x23,delta_y23);
    double distance_13 = hypot(delta_x13,delta_y13);

    double cos_2 = (pow(distance_12,2) + pow(distance_23,2) - pow(distance_13,2))
                    / (2 * distance_12 * distance_23);
    double angle_2 = acos(cos_2);

    last_curve = angle_2/PI;
    last_curve = last_curve * last_curve;
    // last_curve = (last_curve+1)/2;
    // cout << "last_curve " << last_curve << endl;
    if(std::isnan(last_curve)) last_curve = 1;
    if(std::isinf(last_curve)) last_curve = 1;
    if(last_curve > 1) last_curve = 1;
    current_goal.z = last_curve;
  }


  // current_goal.z = 0;
  return;




  
  double lookahead_distance = 5;

  // If nearest sub-goal is not final goal.
  if(min_index < global_path.poses.size() - 1) {

    double p0_x = robot_position.x;
    double p1_x = global_path.poses[min_index].pose.position.x;
    double p2_x = global_path.poses[min_index+1].pose.position.x;
    double p0_y = robot_position.y;
    double p1_y = global_path.poses[min_index].pose.position.y;
    double p2_y = global_path.poses[min_index+1].pose.position.y;

    if(p2_y == p1_y) {
      double pt_x = p0_x;
      double pt_y = p1_y;

      double ptt_x = pt_x + (p2_x - p1_x)/fabs((p2_x - p1_x)) * lookahead_distance;
      double ptt_y = pt_y;

      current_goal.x = ptt_x;
      current_goal.y = ptt_y;
    }
    else if(p2_x == p1_x) {
      double pt_x = p1_x;
      double pt_y = p0_y;

      double ptt_x = pt_x;
      double ptt_y = pt_y  + (p2_y - p1_y)/fabs((p2_y - p1_y)) * lookahead_distance;

      current_goal.x = ptt_x;
      current_goal.y = ptt_y;
    } else {
      double k1 = (p2_y - p1_y) / (p2_x - p1_x);
      double b1 = p1_y - k1 * p1_x;
      double k2 = -1 / k1;
      double b2 = p0_y - k2 * p0_x;

      double pt_y = (b1 + pow(k1,2) * b2) / (1 + pow(k1,2));
      double pt_x = (pt_y - b1) / k1;

      double ptt_x = pt_x + cos(atan2((p2_y - p1_y),(p2_x - p1_x))) * lookahead_distance;
      double ptt_y = pt_y + sin(atan2((p2_y - p1_y),(p2_x - p1_x))) * lookahead_distance;

      current_goal.x = ptt_x;
      current_goal.y = ptt_y;
    }

  } else {
    current_goal.x = global_path.poses[global_path.poses.size()-1].pose.position.x;
    current_goal.y = global_path.poses[global_path.poses.size()-1].pose.position.y;
  }

  if(min_index < global_path.poses.size() - 2) {
    double delta_x12 = global_path.poses[min_index+1].pose.position.x 
                      - global_path.poses[min_index].pose.position.x;
    double delta_x23 = global_path.poses[min_index+2].pose.position.x 
                      - global_path.poses[min_index+1].pose.position.x; 
    double delta_x13 = global_path.poses[min_index+2].pose.position.x 
                      - global_path.poses[min_index].pose.position.x; 
    double delta_y12 = global_path.poses[min_index+1].pose.position.y 
                      - global_path.poses[min_index].pose.position.y;
    double delta_y23 = global_path.poses[min_index+2].pose.position.y 
                      - global_path.poses[min_index+1].pose.position.y;
    double delta_y13 = global_path.poses[min_index+2].pose.position.y 
                      - global_path.poses[min_index].pose.position.y;

    double distance_12 = hypot(delta_x12,delta_y12);
    double distance_23 = hypot(delta_x23,delta_y23);
    double distance_13 = hypot(delta_x13,delta_y13);

    double cos_2 = (pow(distance_12,2) + pow(distance_23,2) - pow(distance_13,2))
                    / (2 * distance_12 * distance_23);
    double angle_2 = acos(cos_2);

    last_curve = angle_2/PI;
    // last_curve = last_curve * last_curve;
    // last_curve = (last_curve+1)/2;
  }
  current_goal.z = last_curve;

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
  pp_command.linear.y = current_goal.z;
  // cout<<"rotation "<<rotation*180/PI<<endl;
  // cout<<"goal_distance "<<goal_distance<<endl;
}
