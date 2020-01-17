#include <Controlling.h>

void Controlling::ComputePurePursuitCommand(geometry_msgs::Point32 Goal_Route, geometry_msgs::Point32 Goal_Plan,geometry_msgs::Twist& Pp_command) {
  double u_turn_speed = 0; 
  double u_turn_rotation = 1.2;
  double rotation_threshold = PI/2;

  double faceing_angle = atan2(Goal_Route.y,Goal_Route.x);
  if(fabs(faceing_angle) > 1 ) {
    Pp_command.linear.x =  speed_scale_ * 0.01;
    Pp_command.angular.z = rotation_scale_ * (faceing_angle/fabs(faceing_angle)) * 0.8;
    return;
  }

  double goal_plan_distance = hypot(Goal_Plan.x,Goal_Plan.y);
  double goal_plan_yaw = atan2(Goal_Plan.y,Goal_Plan.x);

  double temp_index = 0;
  if (goal_plan_yaw < 0) temp_index = 2;

  double rotation = goal_plan_yaw + temp_index*PI;
  if (rotation < -PI) rotation += 2*PI;
  else if (rotation > PI) rotation -= 2*PI;

  if (fabs(rotation) > rotation_threshold) {
    Pp_command.angular.z = (rotation/fabs(rotation)) * u_turn_rotation;
    Pp_command.linear.x = u_turn_speed;
  } else {
    Pp_command.linear.x =  speed_scale_ * goal_plan_distance;
    Pp_command.angular.z = rotation_scale_ * rotation;
  }

}