#include <Controlling.h>

 
/******************************************************************************************
 ****                              Compute Twist Command                               ****
 ******************************************************************************************
 * Modified Pure Pursuit Algorithm for Differential Model                                 *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Goal_Route - Current goal from global route in local frame                         *
 *     Goal_Plan  - Current goal from local path in local frame selected based            * 
 *                  on Lookahead Distance                                                 *
 * Output:                                                                                *
 *     Pp_command  - Twist command linear.x for linear & angular.z for rotation           *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     double u_turn_speed_scale    - scale standard linear speed by set ratio            *
 *     double u_turn_rotation_scale - scale standard rotation speed by set ratio          *
 *     double rotation_threshold    - radian threshold for execute local spin             *
 *   Global:                                                                              *
 *     double speed_scale_          - scale for convert distance to linear speed          *
 *     double rotation_scale_       - scale for convert distance to rotation speed        *
 ******************************************************************************************/
void Controlling::ComputePurePursuitCommand(geometry_msgs::Point32 Goal_Route, geometry_msgs::Point32 Goal_Plan,geometry_msgs::Twist& Pp_command) {
  double u_turn_speed_scale = 0; 
  double u_turn_rotation_scale = 0.8;
  double rotation_threshold = 0.78;
  double face_rotation_threshold = 1.2;

  double goal_plan_distance = hypot(Goal_Plan.x,Goal_Plan.y);
  double goal_plan_yaw = atan2(Goal_Plan.y,Goal_Plan.x);
  double faceing_angle = atan2(Goal_Route.y,Goal_Route.x);

  max_linear_acceleration_ = temp_max_linear_acceleration_;

  if(fabs(faceing_angle) > face_rotation_threshold) {
    Pp_command.linear.x =  speed_scale_ * u_turn_speed_scale;
    Pp_command.angular.z = rotation_scale_ * u_turn_rotation_scale 
                            * (faceing_angle/fabs(faceing_angle));
    max_linear_acceleration_ = temp_max_linear_acceleration_ * 4;
    return;
  }
  
  if(fabs(goal_plan_yaw) > rotation_threshold) {
    Pp_command.linear.x =  speed_scale_ * u_turn_speed_scale;
    Pp_command.angular.z = rotation_scale_ * u_turn_rotation_scale 
                            * (goal_plan_yaw/fabs(goal_plan_yaw));
    max_linear_acceleration_ = temp_max_linear_acceleration_ * 4;
  } else {
    Pp_command.linear.x =  speed_scale_ * goal_plan_distance;
    Pp_command.angular.z = rotation_scale_ * goal_plan_yaw;
  }
}

void Controlling::ComputePurePursuitRRTCommand(geometry_msgs::Point32 Goal_Route, geometry_msgs::Point32 Goal_Plan,geometry_msgs::Twist& Pp_command) {
  double u_turn_speed_scale = 0; 
  double u_turn_rotation_scale = 0.8;
  double rotation_threshold = 0.78;

  double goal_plan_distance = hypot(Goal_Plan.x,Goal_Plan.y);
  double goal_plan_yaw = atan2(Goal_Plan.y,Goal_Plan.x);
  double faceing_angle = atan2(Goal_Route.y,Goal_Route.x);

  if(fabs(goal_plan_yaw) > rotation_threshold) {
    Pp_command.linear.x =  speed_scale_ * u_turn_speed_scale;
    Pp_command.angular.z = rotation_scale_ * u_turn_rotation_scale 
                            * (goal_plan_yaw/fabs(goal_plan_yaw));
  } else {
    Pp_command.linear.x =  speed_scale_ * goal_plan_distance;
    Pp_command.angular.z = rotation_scale_ * goal_plan_yaw;
  }
}