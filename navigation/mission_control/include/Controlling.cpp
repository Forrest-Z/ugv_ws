#include <Controlling.h>

void Controlling::getMovingState(geometry_msgs::Twist Cmd_Vel) {
  // Seting global variable "moving_state_" based on command.
  if (fabs(Cmd_Vel.linear.x) <= 10 * oscillation_ && fabs(Cmd_Vel.angular.z) <= 10 * oscillation_) moving_state_= "stop";
  else if (fabs(Cmd_Vel.linear.x) <= 10 * oscillation_ && fabs(Cmd_Vel.angular.z) > 10 * oscillation_) moving_state_= "rotation";
  else if (fabs(Cmd_Vel.linear.x) > 10 * oscillation_ && fabs(Cmd_Vel.angular.z) <= 10 * oscillation_) moving_state_ = "stright";
  else moving_state_ = "moving";
}

void Controlling::getPredictPath(geometry_msgs::Twist Cmd_Vel) {
  // "point_step" define path points density while showing the path in Rviz.
  double point_step = 0.01;
  path_predict_.points.clear();


  // Drawing circle or lines based on moving state.
  if (moving_state_ == "stop") {
    drawCircle(point_step,vehicle_radius_,path_predict_);
  } 
  else if (moving_state_ == "rotation") {
    drawCircle(point_step,vehicle_radius_,path_predict_);
  } 
  else if (moving_state_ == "stright") {
    geometry_msgs::Point32 origin;
    origin.x = 0;
    double end = Cmd_Vel.linear.x * lookahead_time_;
    origin.y = -vehicle_radius_;
    drawLine(0,end,point_step,origin,path_predict_);
    origin.y = vehicle_radius_;
    drawLine(0,end,point_step,origin,path_predict_);
  } else {
    geometry_msgs::Point32 origin;
    double path_radius = fabs(Cmd_Vel.linear.x/Cmd_Vel.angular.z);
    double inner_radius = path_radius - vehicle_radius_;//path_radius - inflation_rate_ * vehicle_radius_;
    double outer_radius = path_radius + vehicle_radius_;//path_radius + inflation_rate_ * vehicle_radius_;
    double end;
    origin.x = 0;
    if (Cmd_Vel.linear.x >= 0) {
      if (Cmd_Vel.angular.z >= 0) {
        origin.y = path_radius;
        end = 3*PI/2 + Cmd_Vel.linear.x/fabs(Cmd_Vel.linear.x) * Cmd_Vel.angular.z * lookahead_time_/2;
        drawCircle(3*PI/2,end,point_step,inner_radius,origin,path_predict_);
        drawCircle(3*PI/2,end,point_step,outer_radius,origin,path_predict_);
      } else {
        origin.y = -path_radius;
        end = PI/2 + Cmd_Vel.linear.x/fabs(Cmd_Vel.linear.x) * Cmd_Vel.angular.z * lookahead_time_/2;
        drawCircle(PI/2,end,point_step,inner_radius,origin,path_predict_);
        drawCircle(PI/2,end,point_step,outer_radius,origin,path_predict_);
      }
    } else {
      if (Cmd_Vel.angular.z >= 0) {
        origin.y = -path_radius;
        end = PI/2 - Cmd_Vel.linear.x/fabs(Cmd_Vel.linear.x) * Cmd_Vel.angular.z * lookahead_time_/2;
        drawCircle(PI/2,end,point_step,inner_radius,origin,path_predict_);
        drawCircle(PI/2,end,point_step,outer_radius,origin,path_predict_);
      } else {
        origin.y = path_radius;
        end = 3*PI/2 - Cmd_Vel.linear.x/fabs(Cmd_Vel.linear.x) * Cmd_Vel.angular.z * lookahead_time_/2;
        drawCircle(3*PI/2,end,point_step,inner_radius,origin,path_predict_);
        drawCircle(3*PI/2,end,point_step,outer_radius,origin,path_predict_);
      }
    }
  }
}

void Controlling::filterPointClouds(sensor_msgs::PointCloud Cloud_In,geometry_msgs::Twist Cmd_Vel) {
  /*
    "pointcloud_filltered_" is in base frame for pushlish showing in Rviz.
    "pointcloud_onpath_" is in path frame for path based avoidance.
    For most moving state except moving two pointcloud set are same.
    In moving state when vehicle both moving and roatating, 
      predict path would not overlap with current base frame,
      it should be overlap for each predict time pose of vehicle.
    Hence, using Law of cosines to compute the angle between desire point 
      to moving center(origin) and vehicle current pose to moving center.
    Based on distance to path middle line and moving distance following path,
      we could have desire point position in path frame.
  */
  pointcloud_filltered_.points.clear();
  pointcloud_filltered_.header = Cloud_In.header;
  pointcloud_onpath_.points.clear();
  pointcloud_onpath_.header = Cloud_In.header;
  pointcloud_semicircle_.points.clear();
  pointcloud_semicircle_.header = Cloud_In.header;
  pointcloud_obstacle_.points.clear();
  pointcloud_obstacle_.header = Cloud_In.header;  

  if (moving_state_ == "stop") {
    pointcloud_filltered_.points.clear();
    pointcloud_onpath_.points.clear();
  } 
  else if (moving_state_ == "rotation") {
    for (int i = 0; i < Cloud_In.points.size(); ++i) {
      double distance = hypot(Cloud_In.points[i].x,Cloud_In.points[i].y);
      if (distance > 1.5 * vehicle_radius_) continue;
      geometry_msgs::Point32 point;
      point.x = Cloud_In.points[i].x;
      point.y = Cloud_In.points[i].y;
      point.z = Cloud_In.points[i].z;
      pointcloud_filltered_.points.push_back(point);
      pointcloud_onpath_.points.push_back(point);
      pointcloud_semicircle_.points.push_back(point);
      if (Cloud_In.points[i].z != 10) pointcloud_obstacle_.points.push_back(point);
    }
  } 
  else if (moving_state_ == "stright") {
    for (int i = 0; i < Cloud_In.points.size(); ++i) {
      if (fabs(Cloud_In.points[i].y) > inflation_rate_ * vehicle_radius_) continue;
      if (Cmd_Vel.linear.x >= 0) {
        if (Cloud_In.points[i].x > lookahead_time_ * Cmd_Vel.linear.x || Cloud_In.points[i].x < 0) continue;
      } else {
        if (Cloud_In.points[i].x < lookahead_time_ * Cmd_Vel.linear.x || Cloud_In.points[i].x > 0) continue;
      }
      geometry_msgs::Point32 point;
      point.x = Cloud_In.points[i].x;
      point.y = Cloud_In.points[i].y;
      point.z = Cloud_In.points[i].z;
      pointcloud_filltered_.points.push_back(point);
      pointcloud_onpath_.points.push_back(point);
      pointcloud_semicircle_.points.push_back(point);
      if (Cloud_In.points[i].z != 10) pointcloud_obstacle_.points.push_back(point);  
    }
  } else {
    for (int i = 0; i < Cloud_In.points.size(); ++i) {

      double distance = hypot(Cloud_In.points[i].x,Cloud_In.points[i].y);
      double path_radius = fabs(Cmd_Vel.linear.x/Cmd_Vel.angular.z);
      double inner_radius = path_radius - inflation_rate_ * vehicle_radius_;
      double outer_radius = path_radius + inflation_rate_ * vehicle_radius_;
      double point_radius;

      if (Cmd_Vel.linear.x >= 0) {
        if (Cmd_Vel.angular.z >= 0) {
          point_radius = sqrt(pow(Cloud_In.points[i].x,2)+pow((Cloud_In.points[i].y-path_radius),2));
        } else {
          point_radius = sqrt(pow(Cloud_In.points[i].x,2)+pow((Cloud_In.points[i].y+path_radius),2));
        }
      } else {
        if (Cmd_Vel.angular.z >= 0) {
          point_radius = sqrt(pow(Cloud_In.points[i].x,2)+pow((Cloud_In.points[i].y+path_radius),2));
        } else {
          point_radius = sqrt(pow(Cloud_In.points[i].x,2)+pow((Cloud_In.points[i].y-path_radius),2));
        }
      }
      if (point_radius > outer_radius || point_radius < inner_radius) continue;

      if (Cmd_Vel.linear.x > 0 && Cloud_In.points[i].x < 0) continue;
      if (Cmd_Vel.linear.x < 0 && Cloud_In.points[i].x > 0) continue;

      geometry_msgs::Point32 point;

      double numerator = pow(path_radius,2) + pow(point_radius,2) - pow(distance,2);
      double denominator = 2 * path_radius * point_radius;
      double anglePointVehicle = acos(numerator/denominator);

      if(fabs((anglePointVehicle * Cmd_Vel.linear.x)/Cmd_Vel.angular.z) > fabs(Cmd_Vel.linear.x) * lookahead_time_) continue;

      point.x = fabs((anglePointVehicle * Cmd_Vel.linear.x)/Cmd_Vel.angular.z);
      point.y = path_radius - point_radius;
      point.z = Cloud_In.points[i].z;
      pointcloud_onpath_.points.push_back(point);

      if(Cloud_In.points[i].x/Cmd_Vel.linear.x > 0){
        point.x = Cloud_In.points[i].x;
        point.y = Cloud_In.points[i].y;
        point.z = Cloud_In.points[i].z;
        pointcloud_semicircle_.points.push_back(point);        
      }
        
      point.x = Cloud_In.points[i].x;
      point.y = Cloud_In.points[i].y;
      point.z = Cloud_In.points[i].z;
      pointcloud_filltered_.points.push_back(point);
      if (Cloud_In.points[i].z != 10) pointcloud_obstacle_.points.push_back(point);


    }
  }

}

bool Controlling::waitObstaclePass(geometry_msgs::Twist& Cmd_Vel){
  static int waitCount_ = 0;
  if(pointcloud_obstacle_.points.size() == 0){
    waitCount_ = 0;
    return false;
  }
  //Cmd_Vel.angular.z = 0;
  if(waitCount_ >= ROS_RATE_HZ * STOP_TIME_SEC) return false;

  Cmd_Vel.linear.x = 0;
  waitCount_++;
  return true;
}

void Controlling::computeSafePath(geometry_msgs::Twist& Cmd_Vel){
  // Using repulsive force compute on path frame to adjust origin command.
  geometry_msgs::Point32 rep_force;
  computeClearPath(rep_force,Cmd_Vel);
  Cmd_Vel.linear.z = rep_force.x;
  Cmd_Vel.angular.z -= rep_force.y;
  rep_force.x = 0;
  computeSafeSpeed(rep_force,Cmd_Vel);
  Cmd_Vel.linear.x -= rep_force.x;

  if(rep_force.z == 1) Cmd_Vel.linear.x = 0;
}

void Controlling::computeClearPath(geometry_msgs::Point32& Force,geometry_msgs::Twist& Cmd_Vel) {
  // if there are no points on path, return the initial value of force.
  if(pointcloud_filltered_.points.size() == 0){
    return;
  }

  double unit_rad = 1 * PI/180; //0.017453293 = 1 degree
  int sample_half = (PI/2)/unit_rad;
  int sample_num = 2 * sample_half + 1;
  vector<int> path_vector(sample_num);

  int width_index = 10;
  double factor = 0.001;

  int min_safe = -1;

  double min_distance = 10;
  
  for (int i = 0; i < pointcloud_filltered_.points.size(); ++i) {

    double point_rad = atan2(pointcloud_filltered_.points[i].y,pointcloud_filltered_.points[i].x);

    if (point_rad > PI/2) point_rad = PI/2;
    if (point_rad < -PI/2) point_rad = -PI/2;
    int point_ratio = point_rad/unit_rad;

    double distance = hypot(pointcloud_filltered_.points[i].x,pointcloud_filltered_.points[i].y);
    if(distance < min_distance) min_distance = distance;

    int point_width;
    (pointcloud_filltered_.points[i].z == 10) ? point_width = width_index/3 : point_width = width_index;

    // double factor_x;
    // (pointcloud_filltered_.points[i].z == 10) ? factor_x = factor/10 : factor_x = factor; ///pow(distance/5,2)
    // if (fabs(pointcloud_filltered_.points[i].y) < 1.2 * vehicle_radius_) {
    //   Force.x += (factor_x * (fabs(pointcloud_filltered_.points[i].y)+vehicle_radius_/2)/(pow(distance,2)+vehicle_radius_/2));
    // }

    path_vector[point_ratio+sample_half]++;

    for (int j = 0; j < point_width; ++j) {
      if(point_ratio+sample_half+j > sample_num) {
        path_vector[sample_num]++;
      } else {
        path_vector[point_ratio+sample_half+j]++;
      }
      
      if(point_ratio+sample_half-j < 0) {
        path_vector[0]++;
      } else {
        path_vector[point_ratio+sample_half-j]++;
      }
    }

  }

  for (int i = sample_half; i < path_vector.size(); i++) {
    if(path_vector[sample_half-(i-sample_half)] == 0) {
      min_safe = sample_half-(i-sample_half);
      break;
    }
    if(path_vector[sample_half+(i-sample_half)] == 0) {
      min_safe = sample_half+(i-sample_half);
      break;
    }
  }

  if(min_distance < vehicle_radius_){
    ROS_INFO("TOO ClOSE");
    Force.x = 0;
    Force.y = Cmd_Vel.angular.z;
    Force.z = 1;
  }
  else if(min_safe == -1) {
    ROS_INFO("BLOCKED");
    Force.x = 0;
    Force.y = 0;
    Force.z = 1;
  } else {
    // ROS_INFO("Normal");
    double force_factor = -0.2;
    Force.x = min_safe - sample_half;
    Force.y = force_factor * (min_safe - sample_half);
  }

}

void Controlling::computeObstacleForce(geometry_msgs::Point32& Force){
  double factor_y = 0.2;
  for (int i = 0; i < pointcloud_filltered_.points.size(); ++i) {
    double distance = hypot(pointcloud_filltered_.points[i].x,pointcloud_filltered_.points[i].y);
    if (fabs(pointcloud_onpath_.points[i].y) > 2 * vehicle_radius_) continue;
    if (distance > 5 * vehicle_radius_) continue;
    Force.y += (factor_y * pointcloud_filltered_.points[i].y)/pow(distance,2);
  }
}


void Controlling::computeSafeSpeed(geometry_msgs::Point32& Force,geometry_msgs::Twist& Cmd_Vel) {
  // if there are no points on path, return the initial value of force.
  if(pointcloud_filltered_.points.size() == 0){
    return;
  }
  double min_distance = 100;
  for (int i = 0; i < pointcloud_filltered_.points.size(); ++i) {
    if (fabs(pointcloud_onpath_.points[i].y) > 1.5 * vehicle_radius_) continue;
    double distance = hypot(pointcloud_filltered_.points[i].x,pointcloud_filltered_.points[i].y);
    if(distance < min_distance) min_distance = distance;
  }

  if(min_distance < 10) {
    double factor_x = 0.6;
    Force.x = factor_x * Cmd_Vel.linear.x/pow((min_distance/vehicle_radius_),2);
  }
}


/*
void Controlling::computeFromSemi(geometry_msgs::Point32& Force,geometry_msgs::Twist& Cmd_Vel){
  double unit_rad = 1 * PI/180; //0.017453293 = 1 degree
  int sample_half = (PI/2)/unit_rad;
  int sample_num = 2 * sample_half + 1;

  double target_dir = Cmd_Vel.angular.z/unit_rad;

  if (fabs(Cmd_Vel.angular.z) > PI/2) {
    target_dir = (PI/2) * (Cmd_Vel.angular.z/fabs(Cmd_Vel.angular.z))/unit_rad;
  } 

  int min_safe = -1;

  int point_width = 20;
  vector<int> semi_vector(sample_num);

  for (int i = 0; i < pointcloud_semicircle_.points.size(); ++i) {
    double point_rad = atan2(pointcloud_semicircle_.points[i].y,pointcloud_semicircle_.points[i].x);
    if (point_rad > PI/2) point_rad = PI/2;
    if (point_rad < -PI/2) point_rad = -PI/2;
    int point_ratio = point_rad/unit_rad;

    for (int j = 0; j < point_width; ++j) {
      if(point_ratio+sample_half+j > sample_num) {
        semi_vector[sample_num]++;
      } else {
        semi_vector[point_ratio+sample_half+j]++;
      }
      
      if(point_ratio+sample_half-j < 0) {
        semi_vector[0]++;
      } else {
        semi_vector[point_ratio+sample_half-j]++;
      }
    }
  }

  for (int i = sample_half; i < semi_vector.size(); i++) {
    if(semi_vector[sample_half-(i-sample_half)] == 0) {
      min_safe = sample_half-(i-sample_half);
      break;
    }
    if(semi_vector[sample_half+(i-sample_half)] == 0) {
      min_safe = sample_half+(i-sample_half);
      break;
    }
  }

  double force_factor = -0.3;
  Force.x = min_safe - sample_half;
  Force.y = force_factor * (min_safe - sample_half);

}

*/
