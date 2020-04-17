#include <Tools.h>

void Tools::BuildPredictPath(geometry_msgs::Twist Cmd_vel) {
  path_predict_.points.clear();
  path_predict_.header.frame_id = robot_id_ + "/base_link";
  path_predict_.header.stamp = ros::Time::now();
  geometry_msgs::Point32 origin;
  double point_step = 0.01;
  
  GetMovingState(Cmd_vel);
  cmd_vel_ = Cmd_vel;
    
  if (moving_state_ == "stop") {
    DrawCircle(point_step,vehicle_radius_,path_predict_);
  } 
  else if (moving_state_ == "rotation") {
    DrawCircle(point_step,vehicle_radius_,path_predict_);
  } 
  else if (moving_state_ == "stright") {
    origin.x = 0;
    double end = Cmd_vel.linear.x * lookahead_time_;
    origin.y = -vehicle_radius_;
    DrawLine(0,end,point_step,origin,path_predict_);
    origin.y = vehicle_radius_;
    DrawLine(0,end,point_step,origin,path_predict_);
  } else {
    double path_radius = fabs(Cmd_vel.linear.x/Cmd_vel.angular.z);
    double inner_radius = path_radius - vehicle_radius_;
    double outer_radius = path_radius + vehicle_radius_;
    double end;
    origin.x = 0;
    if (Cmd_vel.linear.x >= 0) {
      if (Cmd_vel.angular.z >= 0) {
        origin.y = path_radius;
        end = 3*PI/2 + Cmd_vel.linear.x/fabs(Cmd_vel.linear.x) * Cmd_vel.angular.z * lookahead_time_;
        DrawCircle(3*PI/2,end,point_step,inner_radius,origin,path_predict_);
        DrawCircle(3*PI/2,end,point_step,outer_radius,origin,path_predict_);
      } else {
        origin.y = -path_radius;
        end = PI/2 + Cmd_vel.linear.x/fabs(Cmd_vel.linear.x) * Cmd_vel.angular.z * lookahead_time_;
        DrawCircle(PI/2,end,point_step,inner_radius,origin,path_predict_);
        DrawCircle(PI/2,end,point_step,outer_radius,origin,path_predict_);
      }
    } else {
      if (Cmd_vel.angular.z >= 0) {
        origin.y = -path_radius;
        end = PI/2 - Cmd_vel.linear.x/fabs(Cmd_vel.linear.x) * Cmd_vel.angular.z * lookahead_time_;
        DrawCircle(PI/2,end,point_step,inner_radius,origin,path_predict_);
        DrawCircle(PI/2,end,point_step,outer_radius,origin,path_predict_);
      } else {
        origin.y = path_radius;
        end = 3*PI/2 - Cmd_vel.linear.x/fabs(Cmd_vel.linear.x) * Cmd_vel.angular.z * lookahead_time_;
        DrawCircle(3*PI/2,end,point_step,inner_radius,origin,path_predict_);
        DrawCircle(3*PI/2,end,point_step,outer_radius,origin,path_predict_);
      }
    }
  }
}

sensor_msgs::PointCloud Tools::ConvertVectortoPointcloud(vector<sensor_msgs::PointCloud> Input) {
  sensor_msgs::PointCloud output;
  output.header = Input[0].header;

  geometry_msgs::Point32 point;
  for (int i = 0; i < Input.size(); ++i) {
    for (int j = 0; j < Input[i].points.size(); ++j) {
      point = Input[i].points[j]; 
      output.points.push_back(point);
    }
  }
  return output;
}

void Tools::GetMovingState(geometry_msgs::Twist Cmd_vel){
  if (fabs(Cmd_vel.linear.x) <= oscillation_ && fabs(Cmd_vel.angular.z) <= oscillation_) moving_state_= "stop";
  else if (fabs(Cmd_vel.linear.x) <= oscillation_ && fabs(Cmd_vel.angular.z) > oscillation_) moving_state_= "rotation";
  else if (fabs(Cmd_vel.linear.x) > oscillation_ && fabs(Cmd_vel.angular.z) <= oscillation_) moving_state_ = "stright";
  else moving_state_ = "moving";
}


void Tools::DrawLine(double Begin,double End,double Step,geometry_msgs::Point32 Origin,sensor_msgs::PointCloud& Output) {
  geometry_msgs::Point32 point;
  if(Begin > End) {
    for (double i = Begin; i > End; i-=Step) {
      point.x = Origin.x + i;
      point.y = Origin.y;
      point.z = 0;
      Output.points.push_back(point);
    }
  } else {
    for (double i = Begin; i < End; i+=Step) {
      point.x = Origin.x + i;
      point.y = Origin.y;
      point.z = 0;
      Output.points.push_back(point);
    }
  }
}


void Tools::DrawCircle(double Step,double Radius,sensor_msgs::PointCloud& Output) {
  Step *= PI/(180);
  geometry_msgs::Point32 point;
  for (double i = 0; i < 2*PI; i+=Step) {
    point.x = Radius*cos(i);
    point.y = Radius*sin(i);
    point.z = 0;
    Output.points.push_back(point);
  }
}

void Tools::DrawCircle(double Begin,double End,double Step,double Radius,sensor_msgs::PointCloud& Output) {
  Step *= PI/(180);
  geometry_msgs::Point32 point;
  if(Begin > End) {
    for (double i = Begin; i > End; i-=Step) {
      point.x = Radius*cos(i);
      point.y = Radius*sin(i);
      point.z = 0;
      Output.points.push_back(point);
    }
  } else {
    for (double i = Begin; i < End; i+=Step) {
      point.x = Radius*cos(i);
      point.y = Radius*sin(i);
      point.z = 0;
      Output.points.push_back(point);
    }
  }
}

void Tools::DrawCircle(double Begin,double End,double Step,double Radius,geometry_msgs::Point32 Origin,sensor_msgs::PointCloud& Output) {
  Step *= PI/(180);
  geometry_msgs::Point32 point;
  if(Begin > End) {
    for (double i = Begin; i > End; i-=Step) {
      point.x = Origin.x + Radius*cos(i);
      point.y = Origin.y + Radius*sin(i);
      point.z = 0;
      Output.points.push_back(point);
    }
  } else {
    for (double i = Begin; i < End; i+=Step) {
      point.x = Origin.x + Radius*cos(i);
      point.y = Origin.y + Radius*sin(i);
      point.z = 0;
      Output.points.push_back(point);
    }
  }
}