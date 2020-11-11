 #include <Planning.h>


/******************************************************************************************
 ****                                Generate Safe Path                                ****
 ******************************************************************************************
 * Generate spline path set and choice the best drivable path                             *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Goal        - goal position in local frame                                         *
 *     Obstacle    - fusion obstacle include map and sensor data                          *
 *     Path_Radius - generate candidate paths radius                                      *
 * Output:                                                                                *
 *     Return      - return false when no valid path                                      *
 *     path_best_  - the best drivable path, call path_best() function to obtain path     *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 *     vector<sensor_msgs::PointCloud> path_all_set_  - all candidate path set            *
 *     vector<sensor_msgs::PointCloud> path_safe_set_ - all non collision path set        *
 *     double map_window_radius_                      - radius of local costmap           *
 ******************************************************************************************/
bool Planning::GenerateCandidatePlan(geometry_msgs::Point32 Goal,sensor_msgs::PointCloud Obstacle, double Path_Radius) {
  bool path_result = false;
  mtx_radius_.lock();
  path_window_radius_ = Path_Radius;
  if(path_window_radius_ <= 0) return false;
  if(path_window_radius_ > map_window_radius_) return false;

  if(Path_Radius < 4) {
    ComputeSafePath(costmap_local_,path_set_1_,map_to_path_1_);
    path_result = SelectBestPath(Goal);
  } else {
    ComputeSafePath(costmap_local_,path_set_,map_to_path_);
    path_result = SelectBestPath(Goal);
  }

  mtx_radius_.unlock();
  
  if(!path_result) {
    sensor_msgs::PointCloud path_temp;
    geometry_msgs::Point32 point_temp;
    path_temp.header.frame_id = "/base_link";
    path_temp.header.stamp = ros::Time::now();
    path_temp.points.push_back(point_temp);
    sub_2_path_best_ = path_temp;
    cout << "Spline path find failed !" << endl;
    return false;
  }
  
  return true;
}


/******************************************************************************************
 ****                                  Update Costmap                                  ****
 ******************************************************************************************
 * Create and update costmap every called                                                 *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Obstacle        - fusion obstacle include map and sensor data                      *
 * Output:                                                                                *
 *     Return          - return false when unable to update                               *
 *                       costmap (current nonexistence)                                   *
 *     costmap_local_  - the costmap in local frame, a global nav_msgs::OccupancyGrid     *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 ******************************************************************************************/
bool Planning::UpdateCostmap(sensor_msgs::PointCloud Obstacle) {
  InitLocalCostmap(costmap_local_);
  SetLocalCostmap(costmap_local_,Obstacle);
  ExpandCostmap(costmap_local_,spline_expand_size_);
  return true;
}


/******************************************************************************************
 ****                                 Check Ideal Path                                 ****
 ******************************************************************************************
 * Generate an ideal path (straight line) and check if path drivable                      *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 * 
 *     Goal        - Current goal position in local frame                                 *
 * Output:                                                                                *
 *     Return      - return false when ideal path not drivable                            *
 *     path_best_  - the best drivable path, call path_best() function to obtain path     *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     geometry_msgs::Point32 point_end       - ideal path end point relative to          *
 *                                          path_window_radius_ and current goal position *
 *     double goal_distance                   - distance from vehicle to current goal     *
 *   Global:                                                                              *
 *     double path_window_radius_             - candidate paths radius                    *
 *     nav_msgs::OccupancyGrid costmap_local_ - the costmap in local frame                *
 ******************************************************************************************/
bool Planning::CheckIdealPath(geometry_msgs::Point32& Goal) {
  geometry_msgs::Point32 point_end;
  sensor_msgs::PointCloud path_temp;
  path_temp.header.frame_id = "/base_link";
  path_temp.header.stamp = ros::Time::now();

  double goal_distance = hypot(Goal.x,Goal.y);
  point_end.x = path_window_radius_ * Goal.x / goal_distance;
  point_end.y = path_window_radius_ * Goal.y / goal_distance;

  ComputeStraight(path_temp,point_end);
  if(DetectObstcaleGrid(path_temp,costmap_local_,ADDITION_IDEAL)) {
    path_best_ = path_temp; 
    return true;
  }
  return false;
}


/******************************************************************************************
 ****                                 Select Best Path                                 ****
 ******************************************************************************************
 * Select the best path from all safe paths                                               *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Path_set_2d - safe path set in 2d buffer, stored in spline branch order            *
 *     Goal        - position of current goal in local frame                              *
 * Output:                                                                                *
 *     Return      - return false when safe path set is empty                             *
 *     path_best_  - the best drivable path, call path_best() function to obtain path     *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     static int last_index - path index of last selected path                           *
 *     int max_index         - path index of the best path in current set                 *
 *     int max_size          - path size in current set branch                            *
 *     bool isLastEnable     - value true when the previous selected path is valid        *
 *                             in current set (current not used)                          *
 *     double stable_weight  - addition score scale for previous path (current not used)  *
 *   Global:                                                                              *
 ******************************************************************************************/
bool Planning::SelectBestPath(vector<vector<sensor_msgs::PointCloud>> Path_set_2d,geometry_msgs::Point32 Goal) {
  if(Path_set_2d.size() <= 0) {
    cout << "Candidate Plan Set is Empty" <<endl;
    return false;
  }

  vector<int> path_grade(Path_set_2d.size());
  vector<int> candidate_path_index;
  static int last_index = -1;
  int max_index = -1;
  int max_size = 0;
  bool isLastEnable = false;
  double stable_weight = 0;

  for (int i = 0; i < Path_set_2d.size(); ++i) {
    path_grade[i] = Path_set_2d[i].size();
    if(max_size < Path_set_2d[i].size()) {
      max_size = Path_set_2d[i].size();
      max_index = i;
    }
  }
  if(max_index < 0) return false;

  for (int i = 0; i < path_grade.size(); ++i) {
    if(path_grade[i] == max_size) {
      candidate_path_index.push_back(i);
      if(i == last_index) isLastEnable = true;
    }
  }

  if(candidate_path_index.size() != 1) {
    double min_distance = DBL_MAX;
    for (int i = 0; i < candidate_path_index.size(); ++i) {
      geometry_msgs::Point32 candidate_point = Path_set_2d[candidate_path_index[i]][max_size/2].points[Path_set_2d[candidate_path_index[i]][max_size/2].points.size()/3];
      double path_distance = hypot((candidate_point.y - Goal.y),(candidate_point.x - Goal.x));

      // (max_size > 30) ? stable_weight = 0.1 : stable_weight = 1;
      if(isLastEnable) {
        geometry_msgs::Point32 last_point = Path_set_2d[last_index][max_size/2].points[Path_set_2d[last_index][max_size/2].points.size()/3];
        path_distance += stable_weight * hypot((candidate_point.y - last_point.y),(candidate_point.x - last_point.x));
      }

      if(path_distance < min_distance) {
        min_distance = path_distance;
        max_index = candidate_path_index[i];
      }
    }
  }
  last_index = max_index;
  path_best_ = Path_set_2d[max_index][max_size/2];
  return true;
}


/******************************************************************************************
 ****                              Generate Splines Path                               ****
 ******************************************************************************************
 * Extract joints set to corresponding level of points then generate splines path         *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Joints      - joints set, point level information stored in member .z              *
 * Output:                                                                                *
 *     Path_set    - generated path set                                                   *
 *     Path_set_2d - generated path set in 2d buffer, stored in spline branch order       *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     geometry_msgs::Point32 point_x - joint points for each level                       *
 *   Global:                                                                              *
 ******************************************************************************************/
void Planning::GenerateSplinesPath(vector<sensor_msgs::PointCloud>& Path_set,vector<sensor_msgs::PointCloud> Joints, vector<vector<sensor_msgs::PointCloud>>& Path_set_2d) {
  Path_set.clear();
  Path_set_2d.clear();
  geometry_msgs::Point32 point_0;
  geometry_msgs::Point32 point_1;
  geometry_msgs::Point32 point_2;
  geometry_msgs::Point32 point_3;

  vector<vector<sensor_msgs::PointCloud>> temp_vector_2d(Joints.size());
  sensor_msgs::PointCloud path_temp;
  path_temp.header.frame_id = "/base_link";
  path_temp.header.stamp = ros::Time::now();
  
  for (int i = 0; i < Joints.size(); ++i) {
    for (int j = 0; j < Joints[i].points.size(); ++j) {
      if(int(std::log10(Joints[i].points[j].z)) == 0) point_1 = Joints[i].points[j];
      if(int(std::log10(Joints[i].points[j].z)) == 1) point_2 = Joints[i].points[j];
      if(int(std::log10(Joints[i].points[j].z)) == 2) point_3 = Joints[i].points[j];

      if(ComputeDigit(point_1.z,1) == ComputeDigit(point_2.z,1) && ComputeDigit(point_1.z,1) == ComputeDigit(point_3.z,1) && ComputeDigit(point_2.z,2) == ComputeDigit(point_3.z,2)) {
        point_0.x = point_1.x / 2;  
        point_0.y = point_1.y / 2;  
        ComputeSplines(path_temp,point_1,point_2,point_3,i);
        temp_vector_2d[i].push_back(path_temp);
        Path_set.push_back(path_temp);
      }
    }
  }
  Path_set_2d = temp_vector_2d;
}


/******************************************************************************************
 ****                             Generate Splines Joints                              ****
 ******************************************************************************************
 * Currently set fixed for generating three level of joints                               *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 * Output:                                                                                *
 *     Output - joints set, point level information stored in member .z                   *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     double shrink_scale                         - gap scale between each branch        *
 *     double radius_unit                          - radius unit between each level       *
 *     vector<int> level_nums                      - number of joints in each level       *
 *     vector<double> level_unit                   - gap unit in each level               *
 *     vector<sensor_msgs::PointCloud> temp_vector - temporary stored output vector       *
 *     sensor_msgs::PointCloud temp_level          - temporary stored single level joints *
 *   Global:                                                                              *
 *     double path_window_radius_                  - candidate paths radius               *
 *     double path_swap_range_                     - maximum joint's distribution radian  *
 ******************************************************************************************/
// void Planning::GenerateSplinesJoints(vector<sensor_msgs::PointCloud>& Output) {
//   Output.clear();
//   const int temp_splines_joints_num = 3;

//   double shrink_scale = temp_splines_joints_num * 2;
//   double radius_unit = path_window_radius_ / temp_splines_joints_num;

//   vector<int> level_nums = {5,3,3}; //{9,9,7};
//   vector<double> level_unit(temp_splines_joints_num,0);
//   vector<sensor_msgs::PointCloud> temp_vector(level_nums[0]);

//   level_unit[0] = path_swap_range_ / double(level_nums[0] - 1);
//   for (int i = 1; i < level_unit.size(); ++i) {
//     level_unit[i] = level_unit[i-1] * double(level_nums[i-1] - 1) / (shrink_scale * double(level_nums[i] - 1));
//   }

//   sensor_msgs::PointCloud temp_level;
//   temp_level.header.frame_id = "/base_link";
//   temp_level.header.stamp = ros::Time::now();
//   geometry_msgs::Point32 temp_joint;

//   for (int i = 0; i < temp_vector.size(); ++i) {
//     double level_0_angle = -(path_swap_range_/2) + i*level_unit[0];
//     temp_joint.x = radius_unit * 0.5 * cos(level_0_angle);
//     temp_joint.y = radius_unit * 0.5 * sin(level_0_angle); // 由radius_unit*1 -> radius_unit*0.5, 缩短第一个节点距离
//     temp_joint.z = i+1;
//     if(!DetectPointObstcaleGrid(temp_joint,costmap_local_)) continue;
//     temp_level.points.push_back(temp_joint);
//     for (int j = 0; j < level_nums[1]; ++j) {
//       double level_1_angle = level_0_angle - ((level_nums[1] - 1) * level_unit[1]/2) + j * level_unit[1];
//       temp_joint.x = radius_unit * 1.5 * cos(level_1_angle);
//       temp_joint.y = radius_unit * 1.5 * sin(level_1_angle);
//       temp_joint.z = i+1 + (j+1) * 10;
//       if(!DetectPointObstcaleGrid(temp_joint,costmap_local_)) continue;
//       temp_level.points.push_back(temp_joint);
//       for (int k = 0; k < level_nums[2]; ++k) {
//         double level_2_angle = level_1_angle - ((level_nums[2] - 1) * level_unit[2]/2) + k * level_unit[2];
//         temp_joint.x = radius_unit * 3 * cos(level_2_angle);
//         temp_joint.y = radius_unit * 3 * sin(level_2_angle);
//         temp_joint.z = i+1 + (j+1) * 10 + (k+1) * 100;
//         if(!DetectPointObstcaleGrid(temp_joint,costmap_local_)) continue;
//         temp_level.points.push_back(temp_joint);
//       }
//     }
//     temp_vector[i] = temp_level;
//     temp_level.points.clear();
//   }
//   Output = temp_vector;
// }


/******************************************************************************************
 ****                              Check Path Points Safety                            ****
 ******************************************************************************************
 * Iteration each points in path and check costmap for collision                          *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Path     - 2d vector of all candidate path                                         *
 *     Costmap  - the costmap in local frame                                              *
 *     Addition - addition grid when search obstacle around point                         *
 * Output:                                                                                *
 *     Return   - return false path not safe                                              *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     safety_threshold           - threshold value for determining collision in costmap  *
 *   Global:                                                                              *
 *     int safe_path_search_grid_ - original number of grid when search obstacle          *
 ******************************************************************************************/
bool Planning::DetectObstcaleGrid(sensor_msgs::PointCloud Path,nav_msgs::OccupancyGrid Costmap,int Addition) {
  int search_grid = safe_path_search_grid_ + Addition;
  signed char safety_threshold = 10;
  int grid_size = Costmap.info.width * Costmap.info.height; 
  for (int i = 0; i < Path.points.size(); ++i) {
    if(i % 3 != 0) continue;
    int path_point_index = ConvertCartesianToLocalOccupany(Costmap,Path.points[i]);
    if(path_point_index < 0) continue;
    if(Costmap.data[path_point_index] > safety_threshold) return false;
    for (int j = -search_grid/2; j < search_grid; ++j) {
      for (int k = -search_grid; k < search_grid ; ++k) {
        int check_index = path_point_index + k * Costmap.info.width + j;
        if(!CheckGrid(Costmap,check_index,safety_threshold)) return false;
      }
    }
  }
  return true;
}


/******************************************************************************************
 ****                              Update Costmap Obstacle                             ****
 ******************************************************************************************
 * Update costmap grid value based on fusion obstacle data                                *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Obstacle - fusion obstacle include map and sensor data                             *
 *     Costmap  - initiated costmap canvas                                                *
 * Output:                                                                                *
 *     Costmap  - updated costmap                                                         *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     obstacle_occupancy         - value of obstacle represent in costmap                *
 *   Global:                                                                              *
 *     int safe_path_search_grid_ - original number of grid when search obstacle          *
 *     double map_window_radius_  - radius of local costmap                               *
 ******************************************************************************************/
void Planning::SetLocalCostmap(nav_msgs::OccupancyGrid& Costmap,sensor_msgs::PointCloud Obstacle) {
  signed char obstacle_occupancy = 50;
  int costmap_size = Costmap.info.width * Costmap.info.height;
  for (int i = 0; i < Obstacle.points.size(); ++i) {
    if(fabs(Obstacle.points[i].x) > map_window_radius_ || fabs(Obstacle.points[i].y) > map_window_radius_) continue;
    if(hypot(Obstacle.points[i].x,Obstacle.points[i].y) < 0.2) continue;
    int obstacle_in_map = ConvertCartesianToLocalOccupany(Costmap,Obstacle.points[i]);
    if (obstacle_in_map < 0 || obstacle_in_map > costmap_size) continue;
    Costmap.data[obstacle_in_map] = obstacle_occupancy;
  }
}


/******************************************************************************************
 ****                             Initiate Costmap Obstacle                            ****
 ******************************************************************************************
 * Initiate costmap with preset parameters                                                *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 * Output:                                                                                *
 *     Costmap  - updated costmap                                                         *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     map_origin                 - the origin point position of costmap                  *
 *   Global:                                                                              *
 *     double map_window_radius_  - radius of local costmap                               *
 ******************************************************************************************/
void Planning::InitLocalCostmap(nav_msgs::OccupancyGrid& Costmap) {
  vector<signed char> map_data;
  geometry_msgs::Pose map_origin;

  Costmap.header.frame_id =  "/base_link";
  Costmap.header.stamp = ros::Time::now();
  Costmap.info.resolution = costmap_resolution_;

  Costmap.info.width  = (int) (2 * map_window_radius_) / Costmap.info.resolution;
  Costmap.info.height = (int) (2 * map_window_radius_) / Costmap.info.resolution;

  int costmap_size = Costmap.info.width * Costmap.info.height;
  map_data.resize(costmap_size);
  
  map_origin.position.x = - map_window_radius_;
  map_origin.position.y = - map_window_radius_;
  map_origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  Costmap.info.origin = map_origin;
  Costmap.data = map_data;
}


/******************************************************************************************
 ****                                 Generate Splines                                 ****
 ******************************************************************************************
 * Generate correspond splines path based on input format                                 *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Point_x - joint point from close to far                                            *
 *     Level   - current path belonged level in candidate path set                        *
 * Output:                                                                                *
 *     Output  - single spline path                                                       *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 *     double path_vertical_step_ - path point gap                                        *
 ******************************************************************************************/
/*
void Planning::ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2, double Level) {

  Output.points.clear();
  geometry_msgs::Point32 temp_output;

  double range_x = Point_2.x;
  double range_y = Point_2.y;
  double range   = hypot(range_y,range_x);
  double iter_num = std::floor(range/path_vertical_step_);

  double a_1_numerator = Point_1.x * Point_2.y - Point_2.x * Point_1.y;
  double a_1_denominator = -2 * pow((Point_1.x),2) * Point_2.x * (Point_1.x - Point_2.x);

  double b_1_numerator = pow(Point_1.x,2) * Point_2.y + Point_1.x * Point_2.x * Point_1.y - 2 * pow(Point_2.x,2) * Point_1.y;
  double b_2_numerator = Point_1.x * (Point_2.x * ( Point_1.y - 3 * Point_2.y)) + pow(Point_1.x,2) * Point_2.y + pow(Point_2.x,2) * Point_1.y;
  double b_denominator = 2 * Point_1.x * Point_2.x * (Point_1.x - Point_2.x);

  double a_1 = a_1_numerator / a_1_denominator;
  double b_1 = b_1_numerator / b_denominator;
  double a_2 = a_1 * (Point_1.x / (Point_1.x - Point_2.x));
  double b_2 = b_2_numerator / b_denominator;

  double cubic_value;
  double linear_value;

  for (double i = 0; i < iter_num; i++) {
    temp_output.x = i * path_vertical_step_;

    if(temp_output.x >= 0 && temp_output.x  <= Point_1.x) {
      cubic_value = pow((temp_output.x),3);
      linear_value = temp_output.x;
      temp_output.y = a_1 * cubic_value + b_1 * linear_value;
    }
    else if (temp_output.x >= Point_1.x && temp_output.x  <= Point_2.x) {
      cubic_value = pow((temp_output.x - Point_2.x),3);
      linear_value = temp_output.x - Point_2.x;
      temp_output.y = a_2 * cubic_value + b_2 * linear_value + Point_2.y;
    }
    else {
      // cout << "Unknow Range for X" << endl;
      return;
    }
    temp_output.z = Level;
    Output.points.push_back(temp_output);
  }
}


void Planning::ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2,geometry_msgs::Point32 Point_3,double Level) {

  Output.points.clear();
  geometry_msgs::Point32 temp_output;

  double range_x = Point_3.x;
  double range_y = Point_3.y;
  double range   = hypot(range_y,range_x);
  double iter_num = std::floor(range/path_vertical_step_);

  double m_1 = Point_1.y / Point_1.x;
  double m_2 = (Point_2.y - Point_1.y) / (Point_2.x - Point_1.x);
  double m_3 = (Point_3.y - Point_2.y) / (Point_3.x - Point_2.x);

  double z_2_numerator = m_3 * Point_1.x + m_2 * Point_2.x - m_3 * Point_2.x + 2 * m_2 * Point_3.x + 2 * m_1 * Point_1.x - 2 * m_1 * Point_3.x - 3 * m_2 * Point_1.x;
  double z_3_numerator = m_2 * Point_1.x + m_1 * Point_2.x - m_1 * Point_1.x + 2 * m_3 * Point_2.x -  3 * m_2 * Point_2.x;
  double z_denominator = 4 * Point_2.x * Point_3.x - pow((Point_1.x + Point_2.x),2);
  double z_2 = 6 * z_2_numerator / z_denominator;
  double z_3 = 6 * z_3_numerator / z_denominator;

  double a_1 = z_2 / (6 * (-Point_1.x));
  double b_1 = 2 * z_2 / (6 * Point_1.x);
  double a_2 = (2 * z_2 + z_3) / (6 * (Point_1.x - Point_2.x));
  double b_2 = (2 * z_3 + z_2) / (6 * (Point_2.x - Point_1.x));
  double a_3 = z_3 / (3 * (Point_2.x - Point_3.x));
  double b_3 = z_3 / (6 * (Point_3.x - Point_2.x));

  double linear_value;
  double correction_value;

  for (double i = 0; i <= iter_num; i++) {

    temp_output.x = i * path_vertical_step_;

    if(temp_output.x >= 0 && temp_output.x  <= Point_1.x) {
      linear_value = m_1 * temp_output.x;
      correction_value = a_1 * pow((temp_output.x - Point_1.x),2) * temp_output.x + b_1 *  pow(temp_output.x ,2) * (temp_output.x - Point_1.x);
      temp_output.y = linear_value + correction_value;
    }
    else if (temp_output.x >= Point_1.x && temp_output.x  <= Point_2.x) {
      linear_value = m_2 * (temp_output.x - Point_1.x) + Point_1.y;
      correction_value = a_2 * pow((temp_output.x - Point_2.x),2) * (temp_output.x - Point_1.x) + b_2 *  pow((temp_output.x - Point_1.x),2) * (temp_output.x - Point_2.x);
      temp_output.y = linear_value + correction_value;
    }
    else if (temp_output.x >= Point_2.x && temp_output.x  <= Point_3.x) {
      linear_value = m_3 * (temp_output.x - Point_3.x) + Point_3.y;
      correction_value = a_3 * pow((temp_output.x - Point_3.x),2) * (temp_output.x - Point_2.x) + b_3 *  pow((temp_output.x - Point_2.x),2) * (temp_output.x - Point_3.x);
      temp_output.y = linear_value + correction_value;
    } else {
      // cout << "Unknow Range for X" << endl;
      return;
    }
    temp_output.z = Level;
    Output.points.push_back(temp_output);
  }
}

void Planning::ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_0,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2,geometry_msgs::Point32 Point_3,double Level) {

  Output.points.clear();
  geometry_msgs::Point32 temp_output;

  double range_x = Point_3.x;
  double range_y = Point_3.y;
  double range   = hypot(range_y,range_x);
  double iter_num = std::floor(range/path_vertical_step_);

  double m_1 = (Point_1.y - Point_0.y) / (Point_1.x - Point_0.y);
  double m_2 = (Point_2.y - Point_1.y) / (Point_2.x - Point_1.x);
  double m_3 = (Point_3.y - Point_2.y) / (Point_3.x - Point_2.x);

  double z_2_numerator = m_3 * Point_1.x + m_2 * Point_2.x - m_3 * Point_2.x + 2 * m_2 * Point_3.x + 2 * m_1 * Point_1.x - 2 * m_1 * Point_3.x - 3 * m_2 * Point_1.x;
  double z_3_numerator = m_2 * Point_1.x + m_1 * Point_2.x - m_1 * Point_1.x + 2 * m_3 * Point_2.x -  3 * m_2 * Point_2.x - 2 * m_3 * Point_0.x + 2 * m_2 * Point_0.x;
  double z_denominator = 4 * (Point_0.x * Point_1.x + Point_2.x * Point_3.x - Point_0.x * Point_3.x) - pow((Point_1.x + Point_2.x),2);
  double z_2 = 6 * z_2_numerator / z_denominator;
  double z_3 = 6 * z_3_numerator / z_denominator;

  double a_1 = z_2 / (6 * (Point_0.x - Point_1.x));
  double b_1 = 2 * z_2 / (6 * (Point_1.x - Point_0.x));
  double a_2 = (2 * z_2 + z_3) / (6 * (Point_1.x - Point_2.x));
  double b_2 = (2 * z_3 + z_2) / (6 * (Point_2.x - Point_1.x));
  double a_3 = z_3 / (3 * (Point_2.x - Point_3.x));
  double b_3 = z_3 / (6 * (Point_3.x - Point_2.x));

  double linear_value;
  double correction_value;

  for (double i = 0; i <= iter_num; i++) {

    temp_output.x = i * path_vertical_step_;

    if(temp_output.x >= 0 && temp_output.x  <= Point_1.x) {
      linear_value = m_1 * (temp_output.x - Point_0.x) + Point_0.y;
      correction_value = a_1 * pow((temp_output.x - Point_1.x),2) * (temp_output.x - Point_0.x) + b_1 *  pow((temp_output.x - Point_0.x) ,2) * (temp_output.x - Point_1.x);
      temp_output.y = linear_value + correction_value;
    }
    else if (temp_output.x >= Point_1.x && temp_output.x  <= Point_2.x) {
      linear_value = m_2 * (temp_output.x - Point_1.x) + Point_1.y;
      correction_value = a_2 * pow((temp_output.x - Point_2.x),2) * (temp_output.x - Point_1.x) + b_2 *  pow((temp_output.x - Point_1.x),2) * (temp_output.x - Point_2.x);
      temp_output.y = linear_value + correction_value;
    }
    else if (temp_output.x >= Point_2.x && temp_output.x  <= Point_3.x) {
      linear_value = m_3 * (temp_output.x - Point_3.x) + Point_3.y;
      correction_value = a_3 * pow((temp_output.x - Point_3.x),2) * (temp_output.x - Point_2.x) + b_3 *  pow((temp_output.x - Point_2.x),2) * (temp_output.x - Point_3.x);
      temp_output.y = linear_value + correction_value;
    } else {
      // cout << "Unknow Range for X" << endl;
      return;
    }
    
    temp_output.z = Level;
    Output.points.push_back(temp_output);
  }
}
*/
/******************************************************************************************
 ****                              Generate Straight Line                              ****
 ******************************************************************************************
 * Generate straight line from vehicle to goal in local frame                             *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Goal   - goal position in local frame                                              *
 * Output:                                                                                *
 *     Output - straight line path                                                        *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     iter_num                   - point size in each path                               *
 *   Global:                                                                              *
 *     double path_vertical_step_ - path point gap                                        *
 ******************************************************************************************/
void Planning::ComputeStraight(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Goal) {
  Output.points.clear();
  geometry_msgs::Point32 temp_output;

  double range_x = Goal.x;
  double range_y = Goal.y;
  double range   = hypot(range_y,range_x);
  double iter_num = std::floor(range/path_vertical_step_);
  for (double i = 0; i < iter_num; i++) {
    temp_output.x = i * path_vertical_step_;
    temp_output.y = i * (range_y/iter_num);
    temp_output.z = 10;
    Output.points.push_back(temp_output);
  }

}

bool Planning::DetectPointObstcaleGrid(geometry_msgs::Point32 Point,nav_msgs::OccupancyGrid Costmap) {
  signed char safety_threshold = 10;
  int path_point_index = ConvertCartesianToLocalOccupany(Costmap,Point);
  if(!CheckGrid(Costmap,path_point_index,safety_threshold)) return false;
  return true;
}

//================== Astar Pathfind ========================

bool Planning::UpdateAstarCostmap(sensor_msgs::PointCloud Obstacle) {
  InitAstarLocalCostmap(Astar_local_costmap_);
  SetAstarLocalCostmap(Astar_local_costmap_,Obstacle);
  return true;
}

void Planning::SetAstarLocalCostmap(nav_msgs::OccupancyGrid& Costmap,sensor_msgs::PointCloud Obstacle) {
    signed char obstacle_occupancy = 50;
    int costmap_size = Costmap.info.width * Costmap.info.height;
    for (int i = 0; i < Obstacle.points.size(); ++i) {
        if(fabs(Obstacle.points[i].x) >= Astar_local_map_window_radius_ || 
          fabs(Obstacle.points[i].y) >= Astar_local_map_window_radius_) continue;
        if(hypot(Obstacle.points[i].x,Obstacle.points[i].y) < 0.2) continue;
        int obstacle_in_map = ConvertCartesianToLocalOccupany(Costmap,Obstacle.points[i]);
		    geometry_msgs::Point32 ptr = ConvertCartesianToAstarArray(Obstacle.points[i]);
        // printf("ptr : %d , %d",static_cast<int>(ptr.x),static_cast<int>(ptr.y));
        if(static_cast<int>(ptr.x) < Costmap.info.width && static_cast<int>(ptr.x) < Costmap.info.height)
		      Astar_costmap_array_[static_cast<int>(ptr.x)][static_cast<int>(ptr.y)] = obstacle_occupancy;
        if (obstacle_in_map < 0 || obstacle_in_map > costmap_size) continue;
        Costmap.data[obstacle_in_map] = obstacle_occupancy;
    }
}

void Planning::InitAstarLocalCostmap(nav_msgs::OccupancyGrid& Costmap) {
    vector<signed char> local_search_map_data;
    geometry_msgs::Pose local_search_map_origin;

    Costmap.header.frame_id =  "/base_link";
    Costmap.header.stamp = ros::Time::now();
    Costmap.info.resolution = Astar_local_costmap_resolution_;

    Costmap.info.width  = (int) (2 * Astar_local_map_window_radius_) / Costmap.info.resolution;
    Costmap.info.height = (int) (2 * Astar_local_map_window_radius_) / Costmap.info.resolution;

    int costmap_size = Costmap.info.width * Costmap.info.height;
    local_search_map_data.resize(costmap_size);

    local_search_map_origin.position.x = - Astar_local_map_window_radius_;
    local_search_map_origin.position.y = - Astar_local_map_window_radius_;
    local_search_map_origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    Costmap.info.origin = local_search_map_origin;
    Costmap.data = local_search_map_data;

	  Astar_costmap_array_.resize(Costmap.info.width,vector<signed char>(Costmap.info.height,0));
	  Astar_costmap_array_.assign(Costmap.info.width,vector<signed char>(Costmap.info.height,0));
}

geometry_msgs::Point32 Planning::ConvertCartesianToAstarArray(geometry_msgs::Point32 Point) {
	geometry_msgs::Point32 temp_point;

    temp_point.x = (Point.x - Astar_local_costmap_.info.origin.position.x) / Astar_local_costmap_.info.resolution; //row_num
    temp_point.y = (Point.y - Astar_local_costmap_.info.origin.position.y) / Astar_local_costmap_.info.resolution; //col_num

	return temp_point;
}

geometry_msgs::Point32 Planning::ConvertAstarArrayToCartesian(geometry_msgs::Point32 Point) {
	geometry_msgs::Point32 temp_point;

    temp_point.x = Point.x * Astar_local_costmap_.info.resolution + Astar_local_costmap_.info.origin.position.x; //row_num
    temp_point.y = Point.y * Astar_local_costmap_.info.resolution + Astar_local_costmap_.info.origin.position.y; //col_num

	return temp_point;
}

double Planning::calcAstarG(AstarPoint *temp_start, AstarPoint *point)
{
	double extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? Astar_cost_1 : Astar_cost_2;
	double parentG = point->parent == NULL ? 0 : temp_start->G; //如果是初始节点，则其父节点是空
	return parentG + extraG;
}
 
double Planning::calcAstarH(AstarPoint *point, AstarPoint *start, AstarPoint *end)
{
	//用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
	int search_expand = 2;  //已搜索路径膨胀半径，H置为无穷
	int remove_expand = 4;  //start、end点膨胀半径，H置为绝对
	if((point->x <= start->x + remove_expand && point->x >= start->x - remove_expand && 
		point->y <= start->y + remove_expand && point->y >= start->y - remove_expand) 
		|| (point->x <= end->x + remove_expand && point->x >= end->x - remove_expand 
		&& point->y <= end->y + remove_expand && point->y >= end->y - remove_expand))
	return hypot((double)(end->x - point->x),(double)(end->y - point->y))*Astar_cost_1;

	if(!Astar_all_array_.empty()){
		for(int i = 0; i < Astar_all_array_.size(); i++) {
			for(auto raw_point : Astar_all_array_[i]) {
				if(point->x <= raw_point.x + search_expand && point->x >= raw_point.x - search_expand 
				&& point->y <= raw_point.y + search_expand && point->y >= raw_point.y - search_expand) 
				return INFINITE;
			}
		}
	}
	return hypot((double)(end->x - point->x),(double)(end->y - point->y))*Astar_cost_1;
	// return 0; //Dijkstra 算法
}
 
double Planning::calcAstarF(AstarPoint *point)
{
	return point->G + point->H;
}
 
AstarPoint *Planning::getAstarLeastFpoint()
{
	if (!Astar_open_list_.empty())
	{
		auto resPoint = Astar_open_list_.front();
		for (auto &point : Astar_open_list_)
		if (point->F < resPoint->F)
			resPoint = point;
		return resPoint;
	}
	return NULL;
}
 
AstarPoint *Planning::findAstarPath(AstarPoint &startPoint, AstarPoint &endPoint)
{
	if(!isAstarVaildPoint(&endPoint)){
		cout << "The endPoint was invaild !" << endl;
		return NULL;
	}
	if(startPoint.x == endPoint.x && startPoint.y == endPoint.y){
		cout << "The endPoint coinsided with startPoint !" << endl;
		return NULL;
	}
	Astar_open_list_.push_back(new AstarPoint(startPoint.x,startPoint.y)); 
  int Astar_depth_search = 0;
  
	while (!Astar_open_list_.empty()){
    Astar_depth_search++;
    // cout << "Astar search num : " << Astar_depth_search << endl;
    if(Astar_depth_search >= Astar_depth_limit_) return NULL;

		auto curPoint = getAstarLeastFpoint(); 
		Astar_open_list_.remove(curPoint); 
		Astar_close_list_.push_back(curPoint);

		int row_num = curPoint->x; 
		int col_num = curPoint->y;
		Astar_local_costmap_.data[col_num * Astar_local_costmap_.info.width + row_num] = 10;	

		auto surroundPoints = getAstarSurroundPoints(curPoint);

		for (auto &target : surroundPoints){
			if (!isInAstarList(Astar_open_list_, target)){
 
				target->parent = curPoint;
				target->G = calcAstarG(curPoint, target);
				target->H = calcAstarH(target, &startPoint, &endPoint);
				target->F = calcAstarF(target);
 
				Astar_open_list_.push_back(target);
			}else{
				double tempG = calcAstarG(curPoint, target);
				if (tempG < target->G){
					target->parent = curPoint;
 
					target->G = tempG;
					target->F = calcAstarF(target);
				}
			}
			AstarPoint *resPoint = isInAstarList(Astar_open_list_, &endPoint); //若求最短路径，需要判断endPoint是否在Astar_close_list_
			if (resPoint) return resPoint;
		}

	}
	return NULL;
}
 
vector<sensor_msgs::PointCloud> Planning::getAstarPath(geometry_msgs::Point32 start_point,geometry_msgs::Point32 end_point) {

	geometry_msgs::Point32 start_array = ConvertCartesianToAstarArray(start_point);
	geometry_msgs::Point32 end_array = ConvertCartesianToAstarArray(end_point);

	AstarPoint startPoint(static_cast<int> (start_array.x),static_cast<int> (start_array.y));
	AstarPoint endPoint(static_cast<int> (end_array.x),static_cast<int> (end_array.y));
  
	Astar_path_.clear();
	Astar_all_path_.clear();
  Astar_array_.clear();
  Astar_all_array_.clear();
  
	int search_id = 0;
	int route_branch = 2;
	int push_check = 0;
	

	for(;search_id < Astar_plan_num_;search_id++) {

	  sensor_msgs::PointCloud path_candidate;
    list<AstarPoint> array_candidate;

		if(push_check >= route_branch) break;
		AstarPoint *result = findAstarPath(startPoint, endPoint);
		
		while (result){
      array_candidate.push_back(*result);

			geometry_msgs::Point32 temp_point;
			temp_point.x = result->x;
			temp_point.y = result->y;
			temp_point = ConvertAstarArrayToCartesian(temp_point);
			path_candidate.points.push_back(temp_point);
			
      result = result->parent;
		}

		path_candidate.header.stamp = ros::Time::now();
		path_candidate.header.frame_id = "/base_link";
		Astar_all_path_.push_back(path_candidate);
    Astar_all_array_.push_back(array_candidate);

		if(AstarPathCost(array_candidate)) {
      Astar_array_.push_back(array_candidate);
			Astar_path_.push_back(path_candidate);
			push_check++;
		}
	
		for(list<AstarPoint*>::iterator it = Astar_open_list_.begin();it != Astar_open_list_.end();it++) {
			delete *it;
			*it = NULL;
		}
		for(list<AstarPoint*>::iterator vit = Astar_close_list_.begin();vit != Astar_close_list_.end();vit++) {
			delete *vit;
			*vit = NULL;
		}
		Astar_open_list_.clear();
		Astar_close_list_.clear();
	}

  signed char path_occupancy = 40;
  SetAstarCostmapPath(Astar_array_,path_occupancy);

  if(!Astar_path_.empty()) ROS_INFO("Astar path was found !");
  else ROS_ERROR("Astar path could not be found !");

	return Astar_path_;
}

void Planning::SetAstarCostmapPath(vector<list<AstarPoint> > path_costmap, signed char path_occupancy) {
  if(path_costmap.empty()) return;
	for(auto single_path : path_costmap) {
		for(list<AstarPoint>::iterator iter = single_path.begin(); iter != single_path.end(); iter++) {
			Astar_local_costmap_.data[(*iter).y * Astar_local_costmap_.info.width + (*iter).x] = path_occupancy;
			Astar_costmap_array_[(*iter).x][(*iter).y] = path_occupancy;
		}
	}
}

AstarPoint *Planning::isInAstarList(const list<AstarPoint *> &list, const AstarPoint *point) {
	for (auto p : list)
	if (p->x == point->x && p->y == point->y)
		return p;
	return NULL;
}
 
bool Planning::isAstarCanreach(const AstarPoint *point, const AstarPoint *target) {
	if (target->x < 0 || target->x > Astar_costmap_array_.size() - 1
		|| target->y < 0 || target->y>Astar_costmap_array_[0].size() - 1
		|| Astar_costmap_array_[target->x][target->y] >= 50
		|| (target->x == point->x && target->y == point->y)
		|| isInAstarList(Astar_close_list_, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
	{
		return false;
	}
	else
	{		
		for (int x = target->x - 1; x <= target->x + 1; x++) //判断点四周有障碍物则返回false
		for (int y = target->y - 1; y <= target->y + 1; y++)
		if(x >= 0 && x <= Astar_local_costmap_.info.width-1 && y >= 0 && y <= Astar_local_costmap_.info.height-1)
		if (Astar_costmap_array_[x][y] >= 50) return false;
		return true;

		// if(target->x >= 0 && target->x <= Astar_local_costmap_.info.width-1 && target->y >= 0 && target->y <= Astar_local_costmap_.info.height-1)
		// if (Astar_costmap_array_[target->x][target->y] >= 50) return false;
		// return true;
	}
}
 
vector<AstarPoint *> Planning::getAstarSurroundPoints(const AstarPoint *point) {
	vector<AstarPoint *> surroundPoints;
	for (int x = point->x - 1; x <= point->x + 1; x++){
		for (int y = point->y - 1; y <= point->y + 1; y++){
			if (isAstarCanreach(point,new AstarPoint(x, y)))
				surroundPoints.push_back(new AstarPoint(x, y));
		}
	}
	return surroundPoints;
}

bool Planning::isAstarVaildPoint(const AstarPoint *point) {

	if(point->x >= 0 && point->x <= Astar_local_costmap_.info.width-1 
	  && point->y >= 0 && point->y <= Astar_local_costmap_.info.height-1) {
		return true;
	}
	return false;
}

bool Planning::AstarPathCost(list<AstarPoint> cost_path) {
	list<AstarPoint>::iterator cost_path_first_iter;
	list<AstarPoint>::iterator cost_path_central_iter;
	list<AstarPoint>::iterator cost_path_third_iter;
	list<AstarPoint>::iterator path_first_iter;
	list<AstarPoint>::iterator path_central_iter;
	list<AstarPoint>::iterator path_third_iter;
	if(cost_path.empty()) return false;
	if(!Astar_array_.empty()) {
		int i = 0;
		for(list<AstarPoint>::iterator iter = cost_path.begin();iter != cost_path.end();iter++) {
			i++;
			if(i == cost_path.size()/4) {
				cost_path_first_iter = iter;
			}
			if(i == cost_path.size()/2) {
				cost_path_central_iter = iter;
			}
			if(i == cost_path.size()*3/4) {
				cost_path_third_iter = iter;
			}

		}
		for(int j = 0;j < Astar_array_.size();j++){
			int k = 0;
			for(list<AstarPoint>::iterator viter = Astar_array_[j].begin();viter != Astar_array_[j].end();viter++) {
				k++;
				if(k == Astar_array_[j].size()/4) {
					path_first_iter = viter;
				}
				if(k == Astar_array_[j].size()/2) {
					path_central_iter = viter;
				}
				if(k == Astar_array_[j].size()*3/4) {
					path_third_iter = viter;
				}
			}
			int check_value = 0;
			int x_min,x_max,y_min,y_max,x_test,y_test;

      int x_first_1 = (*cost_path_first_iter).x;
      int x_first_2 = (*path_first_iter).x;
      int y_first_1 = (*cost_path_first_iter).y;
      int y_first_2 = (*path_first_iter).y;
      x_min = (x_first_1 < x_first_2) ? x_first_1 : x_first_2;
      x_max = (x_first_1 > x_first_2) ? x_first_1 : x_first_2;
			y_min = (y_first_1 < y_first_2) ? y_first_1 : y_first_2;
      y_max = (y_first_1 > y_first_2) ? y_first_1 : y_first_2;
      for(x_test = x_min; x_test <= x_max; x_test++) {
        for(y_test = y_min; y_test <= y_max; y_test++) {
          if(Astar_costmap_array_[x_test][y_test] >= 50) {
            check_value++;
            break;
          }
        }
      }

			int x_central_1 = (*cost_path_central_iter).x;
      int x_central_2 = (*path_central_iter).x;
      int y_central_1 = (*cost_path_central_iter).y;
      int y_central_2 = (*path_central_iter).y;
      x_min = (x_central_1 < x_central_2) ? x_central_1 : x_central_2;
      x_max = (x_central_1 > x_central_2) ? x_central_1 : x_central_2;
      y_min = (y_central_1 < y_central_2) ? y_central_1 : y_central_2;
      y_max = (y_central_1 > y_central_2) ? y_central_1 : y_central_2;
      for(x_test = x_min; x_test <= x_max; x_test++) {
        for(y_test = y_min; y_test <= y_max; y_test++) {
          if(Astar_costmap_array_[x_test][y_test] >= 50) {
            check_value++;
            break;
          }
        }
      }

			int x_third_1 = (*cost_path_third_iter).x;
      int x_third_2 = (*path_third_iter).x;
      int y_third_1 = (*cost_path_third_iter).y;
      int y_third_2 = (*path_third_iter).y;
      x_min = (x_third_1 < x_third_2) ? x_third_1 : x_third_2;
      x_max = (x_third_1 > x_third_2) ? x_third_1 : x_third_2;
      y_min = (y_third_1 < y_third_2) ? y_third_1 : y_third_2;
      y_max = (y_third_1 > y_third_2) ? y_third_1 : y_third_2;
      for(x_test = x_min; x_test <= x_max; x_test++) {
        for(y_test = y_min; y_test <= y_max; y_test++) {
          if(Astar_costmap_array_[x_test][y_test] >= 50) {
            check_value++;
            break;
          }
        }
      }
			if(check_value == 0) return false;
		}
		
	}
	return true;
}

//============ RRT Pathfind ===============================
nav_msgs::Path Planning::getRRTPlan(const geometry_msgs::Point32 start, const geometry_msgs::Point32 goal) {      
  RRT_path_que_.clear();
  RRT_costmap_local_ = costmap_local_; //costmap_local在二代样条寻路时已经膨胀spline_expand_size;
  // RRTExpandCostmap(RRT_costmap_local_);
  
  for(int search_times = 0;search_times < RRT_search_plan_num_;search_times++) {
    RRTInitialRoot(start);

    if(BuildRRT(start,goal)) {
      ROS_INFO("RRT plan was found !");
      SetRRTPath();
      SetRRTTree();
    } else {
      ROS_ERROR("RRT plan could not be found in %d cycles.", RRT_iteration_);
      SetRRTTree();
      RRT_path_.poses.clear();
    }

    RRTClear();
  }

  RRT_Curve_path_ = ComputeBSplineCurve(RRT_path_);
  return RRT_Curve_path_;
}

bool Planning::BuildRRT(geometry_msgs::Point32 start,geometry_msgs::Point32 goal) {
  for(int i = 0; i < RRT_iteration_; i++){
    RRTProbSample(goal);
    for(int j = 0; j < RRT_current_sampling_.size(); j++){
      if(RRTExtendNearestNode(RRT_current_sampling_[j],goal))
        return true;
      else if(RRTEndAhead(start,goal,RRT_nodes_.back())) 
        return false;
    } 
  }
  return false;
}

void Planning::SetRRTPath() {
    RRT_path_.poses.clear();
    geometry_msgs::PoseStamped pose;
    
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/base_link";
    pose.pose.position.x = RRT_goal_node_->cell.x;
    pose.pose.position.y = RRT_goal_node_->cell.y;
    pose.pose.orientation.w = 1;
    RRT_path_.poses.push_back(pose);
    RRTTreeNode* father = RRT_goal_node_->father;
    while(father != NULL)
    {
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/base_link";
        pose.pose.position.x = father->cell.x;
        pose.pose.position.y = father->cell.y;
        pose.pose.orientation.w = 1;
        RRT_path_.poses.push_back(pose);
        father = father->father;
    }
    if(!RRT_path_.poses.empty()) {
        RRT_path_.header.stamp = RRT_path_.poses[0].header.stamp;
        RRT_path_.header.frame_id = RRT_path_.poses[0].header.frame_id;
    }
    RRT_path_que_.push_back(RRT_path_);
}

void Planning::SetRRTTree() {
    RRT_tree_.points.clear();
    RRT_tree_.header.frame_id = "/base_link";
    for(int i = 0; i < RRT_nodes_.size(); i++)
    {
        geometry_msgs::Point32 point;
        point.x = RRT_nodes_[i]->cell.x;
        point.y = RRT_nodes_[i]->cell.y;
        RRT_tree_.points.push_back(point);
    }
}
 
bool Planning::RRTExtendNearestNode(RRTPointCell random,geometry_msgs::Point32 goal) {
    int j;
    double path_length = DBL_MAX;
    for(int i = 0; i < RRT_nodes_.size(); i++)
    {
      if(hypot(RRT_nodes_[i]->cell.x - random.x, RRT_nodes_[i]->cell.y - random.y) <= path_length)
      {
        path_length = hypot(RRT_nodes_[i]->cell.x - random.x, RRT_nodes_[i]->cell.y - random.y);
        j = i;
      }
    }

    RRTTreeNode* node = new RRTTreeNode;
    if(RRTExtendSingleStep(RRT_nodes_[j],node,random,goal))
    {
      node->father = RRT_nodes_[j];
      RRT_nodes_[j]->children.push_back(node);
      RRT_nodes_.push_back(node);
      if(RRTGoalReached(node,goal))
      {
          RRT_goal_node_ = node;
          return true;
      }
    }
    return false;
}

bool Planning::RRTExtendSingleStep(const RRTTreeNode* rrtnode, RRTTreeNode* &node, const RRTPointCell random, geometry_msgs::Point32 goal) {
    if(random.x == rrtnode->cell.x && random.y == rrtnode->cell.y) return false;
    double sintheta, costheta;
    geometry_msgs::Point32 check_point;
    sintheta = (random.y - rrtnode->cell.y) / hypot(random.x - rrtnode->cell.x, random.y - rrtnode->cell.y);
    costheta = (random.x - rrtnode->cell.x) / hypot(random.x - rrtnode->cell.x, random.y - rrtnode->cell.y);
    node->cell.x = rrtnode->cell.x + RRT_extend_step_ * costheta;
    node->cell.y = rrtnode->cell.y + RRT_extend_step_ * sintheta;
    check_point.x = node->cell.x;
    check_point.y = node->cell.y;

    if(hypot(goal.x - check_point.x,goal.y - check_point.y) < 0.5) return true; //目标点2m以内障碍物忽略

    int check_id = ConvertCartesianToLocalOccupany(RRT_costmap_local_,check_point);
    if(RRT_costmap_local_.data[check_id] >= 50) {
        delete node;
        return false;
    }
    else
        return true;
}
 
void Planning::RRTProbSample(geometry_msgs::Point32 goal) {    
    double prob = RRT_rd_() % 10 * 0.1;
    if(prob < RRT_probability_) {
        RRTAddGoalOrientNode(goal);
    } else RRTAddRandomNode();
}
 
void Planning::RRTAddGoalOrientNode(geometry_msgs::Point32 goal) {
  RRT_current_sampling_.clear();
  RRTPointCell p;
  //这里我们可以将撒的点直接设在终点，也可以在终点附近有一个高斯分布
  p.x = goal.x;// + (RRT_rd_() % 200 - 100) * 0.2;
  p.y = goal.y;// + (RRT_rd_() % 200 - 100) * 0.2;
  RRT_current_sampling_.push_back(p);
}
 
void Planning::RRTAddRandomNode() {
    RRT_current_sampling_.clear();
    for(int i = 0;i < RRT_sample_num_;i++) {
        int RRT_rd_range = map_window_radius_;
        double x = (RRT_rd_() % (2 * RRT_rd_range - 1) + (RRT_rd_() % 9) * 0.1 + (RRT_rd_() % 9) * 0.01) - RRT_rd_range;
        double y = (RRT_rd_() % (2 * RRT_rd_range - 1) + (RRT_rd_() % 9) * 0.1 + (RRT_rd_() % 9) * 0.01) - RRT_rd_range;
        
        RRTPointCell p;
        p.x = x;
        p.y = y;
        
        RRT_current_sampling_.push_back(p);
    }
}
 
bool Planning::RRTGoalReached(const RRTTreeNode* nearestNode, geometry_msgs::Point32 goal) {
    if(hypot(nearestNode->cell.x - goal.x,nearestNode->cell.y - goal.y) < RRT_threhold_)
        return true;
    else
        return false;
}
 
void Planning::RRTInitialRoot(const geometry_msgs::Point32 start) {
    RRT_root_ = new RRTTreeNode;
    RRT_root_->father = NULL;
    RRT_root_->children.clear();

    RRT_nodes_.clear();
    RRT_root_->cell.x = start.x;
    RRT_root_->cell.y = start.y;
    RRT_nodes_.push_back(RRT_root_);
}

void Planning::RRTExpandCostmap(nav_msgs::OccupancyGrid &Grid) {
    Grid = costmap_local_;
    for(int index = 0; index < Grid.data.size(); index++) {
        if(Grid.data[index] == 50) {
            int col_num = index / Grid.info.width; //width 是x方向的长度
  	        int row_num = index % Grid.info.width;
            for(int i = col_num - RRT_expand_size_; i <= col_num + RRT_expand_size_; i++) {
                if(i < 0 || i > Grid.info.height - 1) continue;
                for(int j = row_num - RRT_expand_size_; j <= row_num + RRT_expand_size_; j++) {
                    if(j < 0 || j > Grid.info.width - 1) continue;
                    if(Grid.data[i * Grid.info.width + j] != 50)
                    Grid.data[i * Grid.info.width + j] = 55;
                }
            }

        } 
    }
}

nav_msgs::Path Planning::ComputeBSplineCurve(nav_msgs::Path path) {
    vector<geometry_msgs::Point32> spline_points;
    geometry_msgs::Point32 temp_point;
    nav_msgs::Path temp_path;
    geometry_msgs::PoseStamped temp_path_point; 
    int index = 1;

    if(path.poses.size() < 2) return temp_path;

    spline_points.push_back(temp_point); //预留一个位置

    temp_point.x = path.poses.front().pose.position.x;
    temp_point.y = path.poses.front().pose.position.y;
    spline_points.push_back(temp_point);

    for(; index < path.poses.size() - 1; index++) {
       if(index % 5 == 0) {
            temp_point.x = path.poses[index].pose.position.x;
            temp_point.y = path.poses[index].pose.position.y;
            spline_points.push_back(temp_point);
        } 
    }

    temp_point.x = path.poses.back().pose.position.x;
    temp_point.y = path.poses.back().pose.position.y;
    spline_points.push_back(temp_point);


    temp_point.x = spline_points.back().x * 2 -  spline_points[spline_points.size()-2].x;
    temp_point.y = spline_points.back().y * 2 -  spline_points[spline_points.size()-2].y;
    spline_points.push_back(temp_point);

    spline_points.front().x = spline_points[1].x * 2 - spline_points[2].x; 
    spline_points.front().y = spline_points[1].y * 2 - spline_points[2].y; 

    for(int i = 0; i < spline_points.size() - 3; i++) {
        double t = 0;
        double a_0 = (spline_points[i].x + 4 * spline_points[i+1].x + spline_points[i+2].x) / 6;
        double a_1 = -(spline_points[i].x - spline_points[i+2].x) / 2;
        double a_2 = (spline_points[i].x - 2 * spline_points[i+1].x + spline_points[i+2].x) / 2;
        double a_3 = -(spline_points[i].x - 3 * spline_points[i+1].x + 3 * spline_points[i+2].x - spline_points[i+3].x) / 6;

        double b_0 = (spline_points[i].y + 4 * spline_points[i+1].y + spline_points[i+2].y) / 6;
        double b_1 = -(spline_points[i].y - spline_points[i+2].y) / 2;
        double b_2 = (spline_points[i].y - 2 * spline_points[i+1].y + spline_points[i+2].y) / 2;
        double b_3 = -(spline_points[i].y - 3 * spline_points[i+1].y + 3 * spline_points[i+2].y - spline_points[i+3].y) / 6;

        
        while(t <= 1) {
            temp_path_point.header.frame_id = "/base_link";
            temp_path_point.header.stamp = ros::Time::now();

            temp_path_point.pose.position.x = a_0 + a_1 * t + a_2 * pow(t,2) + a_3 * pow(t,3);
            temp_path_point.pose.position.y = b_0 + b_1 * t + b_2 * pow(t,2) + b_3 * pow(t,3);
            temp_path.poses.push_back(temp_path_point);
            t = t + 0.1;

        }
    }
    temp_path.header.frame_id = temp_path.poses.front().header.frame_id;
    temp_path.header.stamp = temp_path.poses.front().header.stamp;

    return temp_path;
}

bool Planning::RRTEndAhead(geometry_msgs::Point32 start_point,geometry_msgs::Point32 goal_point,RRTTreeNode* grow_point) {
  static int index;
  bool end_ahead;
  if(history_grow_projection_que_.empty()) history_grow_projection_que_.push(-9999);

  double start_grow_length = hypot(grow_point->cell.y - start_point.y,grow_point->cell.x - start_point.x);
  double start_goal_length = hypot(goal_point.y - start_point.y,goal_point.x - start_point.x);
  double grow_goal_length = hypot(goal_point.y - grow_point->cell.y,goal_point.x - grow_point->cell.x);

  double costheta = (pow(start_grow_length,2) + pow(start_goal_length,2) - pow(grow_goal_length,2)) / (2 * start_grow_length * start_goal_length);

  // 生长点在起始点与终点连线上的投影，正负表方向
  double grow_projection_distance = start_grow_length * costheta;

  // 查看priority_que，并重新push
  // vector<double> temp_que;
  // while(!history_grow_projection_que_.empty()) {
  //   temp_que.push_back(history_grow_projection_que_.top());
  //   cout << history_grow_projection_que_.top() << " , ";
  //   history_grow_projection_que_.pop(); 
  // }
  // cout << endl;
  // for(int i = 0; i < temp_que.size(); i++) {
  //   history_grow_projection_que_.push(temp_que[i]);
  // }

  if(grow_projection_distance <= history_grow_projection_que_.top()) index++;
  else index = 0;

  if(index >= 100) {
    end_ahead = true;
    ClearHistoryGrowQue();
  }
  else end_ahead = false;
  history_grow_projection_que_.push(grow_projection_distance);

  return end_ahead;

} 

// local multi-spline pathfind
void Planning::InitSplineCurve(string Spline_folder,string Spline_name) {
  joints_set_.clear();
  path_set_.clear();

  InitLocalCostmap(costmap_local_);
  GenerateSplinesJoints(joints_set_,path_window_radius_);
  GenerateSplinesPath(path_set_,joints_set_);
  GenerateRevoluteSplinePath(path_set_);
  if(!ReadAdjacentListTXT(Spline_folder,Spline_name,map_to_path_)) return;
}

void Planning::InitSplineCurve1st(string Spline_folder,string Spline_name) {
  joints_set_.clear();
  path_set_1_.clear();

  InitLocalCostmap(costmap_local_);
  GenerateSplinesJoints(joints_set_,path_window_radius_1_);
  GenerateSplinesPath(path_set_1_,joints_set_);
  GenerateRevoluteSplinePath(path_set_1_);
  if(!ReadAdjacentListTXT(Spline_folder,Spline_name,map_to_path_1_)) return;
}

void Planning::GenerateSplinesJoints(vector<sensor_msgs::PointCloud>& Output,double Path_Radius) {
  Output.clear();
  const int temp_splines_joints_num = 3;

  double first_shrink_scale = temp_splines_joints_num * 0.95;
  double second_shrink_scale = temp_splines_joints_num * 0.55;
  double radius_unit = Path_Radius / temp_splines_joints_num;

  vector<int> level_nums = {spline_1st_level_,spline_2nd_level_,spline_3rd_level_}; //{1,7,5};
  vector<double> level_unit(temp_splines_joints_num,0);
  vector<sensor_msgs::PointCloud> temp_vector(level_nums[0]);

  if(level_nums[0] == 1) level_unit[0] = 0;
  else level_unit[0] = path_swap_range_ / double(level_nums[0] - 1);

  level_unit[1] = path_swap_range_ / (first_shrink_scale * double(level_nums[1] - 1));
  level_unit[2] = level_unit[1] * double(level_nums[1] - 1) / (second_shrink_scale * double(level_nums[2] - 1));

  sensor_msgs::PointCloud temp_level;
  temp_level.header.frame_id = "/base_link";
  temp_level.header.stamp = ros::Time::now();
  geometry_msgs::Point32 temp_joint;

  for (int i = 0; i < temp_vector.size(); ++i) {
    double level_0_angle = 0;
    temp_joint.x = radius_unit * 0.5* cos(level_0_angle);
    temp_joint.y = radius_unit * 0.5 * sin(level_0_angle); // 由radius_unit*1 -> radius_unit*0.5, 缩短第一个节点距离
    temp_joint.z = i+1;
    temp_level.points.push_back(temp_joint);
    for (int j = 0; j < level_nums[1]; ++j) {
      double level_1_angle = level_0_angle - ((level_nums[1] - 1) * level_unit[1]/2) + j * level_unit[1];
      temp_joint.x = radius_unit * 1.5 * cos(level_1_angle);
      temp_joint.y = radius_unit * 1.5 * sin(level_1_angle);
      temp_joint.z = i+1 + (j+1) * 10;
      temp_level.points.push_back(temp_joint);
      for (int k = 0; k < level_nums[2]; ++k) {
        double level_2_angle = level_1_angle - ((level_nums[2] - 1) * level_unit[2]/2) + k * level_unit[2];
        temp_joint.x = radius_unit * 3 * cos(level_2_angle);
        temp_joint.y = radius_unit * 3 * sin(level_2_angle);
        temp_joint.z = i+1 + (j+1) * 10 + (k+1) * 100;
        temp_level.points.push_back(temp_joint);
      }
    }
    temp_vector[i] = temp_level;
    temp_level.points.clear();
  }
  Output = temp_vector;

  for(int i = 0; i < Output.size(); i++) {
    for(int j = 0; j < Output[i].points.size(); j++) {
      cout << Output[i].points[j].x << " " << Output[i].points[j].y << endl;
    }
  }
}

void Planning::GenerateSplinesPath(vector<PathGroup>& Path_set,vector<sensor_msgs::PointCloud> Joints) {
  Path_set.clear();
  geometry_msgs::Point32 point_0;
  geometry_msgs::Point32 point_1;
  geometry_msgs::Point32 point_2;
  geometry_msgs::Point32 point_3;

  vector<vector<sensor_msgs::PointCloud>> temp_vector_2d(Joints.size());
  sensor_msgs::PointCloud path_temp;
  PathGroup path_group_temp;
  path_temp.header.frame_id = "/base_link";
  path_temp.header.stamp = ros::Time::now();
  
  for (int i = 0; i < Joints.size(); ++i) {
    for (int j = 0; j < Joints[i].points.size(); ++j) {
      if(int(std::log10(Joints[i].points[j].z)) == 0) point_1 = Joints[i].points[j];
      if(int(std::log10(Joints[i].points[j].z)) == 1) point_2 = Joints[i].points[j];
      if(int(std::log10(Joints[i].points[j].z)) == 2) point_3 = Joints[i].points[j];

      if(ComputeDigit(point_1.z,1) == ComputeDigit(point_2.z,1) && ComputeDigit(point_1.z,1) == ComputeDigit(point_3.z,1) && ComputeDigit(point_2.z,2) == ComputeDigit(point_3.z,2)) {
        point_0.x = point_1.x / 2;  
        point_0.y = point_1.y / 2;  
        ComputeSplines(path_temp,point_1,point_2,point_3,i);

        path_group_temp.path_pointcloud = path_temp;
        path_group_temp.path_pointcloud.channels.resize(1);
        path_group_temp.path_pointcloud.channels[0].name = "path_id";
        path_group_temp.path_pointcloud.channels[0].values.resize(path_temp.points.size(),1);
        path_group_temp.path_id = id_num_;
        id_num_++;
        temp_vector_2d[i].push_back(path_temp);
        Path_set.push_back(path_group_temp);
      }
    }
  }
}

void Planning::ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2,geometry_msgs::Point32 Point_3,double Level) {

  Output.points.clear();
  geometry_msgs::Point32 temp_output;
  geometry_msgs::Point32 Point_0;
  vector<geometry_msgs::Point32> Point_group;

  vector<double> path_h(3,0);
  vector<double> path_a(3,0);
  vector<double> path_b(3,0);
  vector<double> path_c(3,0);
  vector<double> path_d(3,0);
  vector<double> path_m(4,0);

  bool x_input;
  bool variable_increase;

  double temp_p_1;
  double temp_p_2;
  
  if(Point_1.x > 0 && Point_1.x < Point_2.x && Point_2.x < Point_3.x) { 
    x_input = true; 
    variable_increase = true; 
    Point_group.push_back(Point_0);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_3);

  } else if(Point_1.x < 0 && Point_1.x > Point_2.x && Point_2.x > Point_3.x) { 
    x_input = true; 
    variable_increase = false; 
    Point_group.push_back(Point_3);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_0);

  } else if(Point_1.y > 0 && Point_1.y < Point_2.y && Point_2.y < Point_3.y) { 
    x_input = false; 
    variable_increase = true; 
    Point_group.push_back(Point_0);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_3);

  } else if(Point_1.y < 0 && Point_1.y > Point_2.y && Point_2.y > Point_3.y) { 
    x_input = false; 
    variable_increase = false; 
    Point_group.push_back(Point_3);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_0);

  }

  if(x_input && variable_increase ) {

    path_h[0] = Point_group[1].x - Point_group[0].x;
    path_h[1] = Point_group[2].x - Point_group[1].x;
    path_h[2] = Point_group[3].x - Point_group[2].x;

    temp_p_1 = 6 * ((Point_group[2].y - Point_group[1].y)/path_h[1] - (Point_group[1].y - Point_group[0].y)/path_h[0]);
    temp_p_2 = 6 * ((Point_group[3].y - Point_group[2].y)/path_h[2] - (Point_group[2].y - Point_group[1].y)/path_h[1]);


    path_m[0] = 0;
    path_m[2] = ((path_h[0]+path_h[1])*temp_p_2*2 - path_h[1]*temp_p_1) / (4*(path_h[0]+path_h[1])*(path_h[1]+path_h[2]) - pow(path_h[1],2));
    path_m[1] = (temp_p_1 - path_h[1]*path_m[2]) / (2*(path_h[0]+path_h[1]));
    path_m[3] = 0;

    for(int i = 0; i < path_h.size(); i++) {
      path_a[i] = Point_group[i].y;
      path_b[i] = (Point_group[i+1].y - Point_group[i].y)/path_h[i] - path_h[i]*path_m[i]/2 - path_h[i]*(path_m[i+1] - path_m[i])/6;
      path_c[i] = path_m[i]/2;
      path_d[i] = (path_m[i+1] - path_m[i])/path_h[i]/6;
    }

    double range   = fabs(Point_3.x);
    double iter_num = 15;
    path_vertical_step_ = range / iter_num;

    for (int j = 0; j <= iter_num; j++) {
      temp_output.x = j * path_vertical_step_;
      if(temp_output.x >= Point_group[0].x && temp_output.x  <= Point_group[1].x) {
        temp_output.y = path_a[0] + path_b[0]*(temp_output.x - Point_group[0].x) + path_c[0]*pow(temp_output.x - Point_group[0].x,2) + path_d[0]*pow(temp_output.x - Point_group[0].x,3);
      }
      else if (temp_output.x >= Point_group[1].x && temp_output.x  <= Point_group[2].x) {
        temp_output.y = path_a[1] + path_b[1]*(temp_output.x - Point_group[1].x) + path_c[1]*pow(temp_output.x - Point_group[1].x,2) + path_d[1]*pow(temp_output.x - Point_group[1].x,3);
      }
      else if (temp_output.x >= Point_group[2].x && temp_output.x  <= Point_group[3].x) {
        temp_output.y = path_a[2] + path_b[2]*(temp_output.x - Point_group[2].x) + path_c[2]*pow(temp_output.x - Point_group[2].x,2) + path_d[2]*pow(temp_output.x - Point_group[2].x,3);
      } else {
        continue;
      }
      
      temp_output.z = spline_height_;
      Output.points.push_back(temp_output);
    }
    spline_height_+=0.1;
  }

}

void Planning::GenerateRevoluteSplinePath(vector<PathGroup> &Path_set) {
  double average_angle = 10.0/180*PI;
  int single_num = spline_array_num_ / 2;
  double shrink_rate = -average_angle * single_num;
  double z_height = 10;
  vector<PathGroup> Path_set_init = Path_set;
  double path_scale = 1000;

  for(int index = 0;index < spline_array_num_;index++) {
    double revolute_angle = shrink_rate;
    shrink_rate += average_angle;
    if(index == spline_array_num_/2) continue;

    for(int i = 0; i < Path_set_init.size(); i++) {
      PathGroup Path_sub_path;
      for(int j = 0; j < Path_set_init[i].path_pointcloud.points.size(); j++) {
        geometry_msgs::Point32 temp_point;
        if(j == 0) {
          temp_point.x = 0;
          temp_point.y = 0;
          temp_point.z = Path_set_init[i].path_pointcloud.points[j].z + z_height; ;
        } else {
          double point_distance = hypot(Path_set_init[i].path_pointcloud.points[j].x,Path_set_init[i].path_pointcloud.points[j].y);
          double costheta = Path_set_init[i].path_pointcloud.points[j].x / point_distance;
          double sintheta = Path_set_init[i].path_pointcloud.points[j].y / point_distance;

          temp_point.x = point_distance * (costheta*cos(revolute_angle)-sintheta*sin(revolute_angle));
          temp_point.y = point_distance * (sintheta*cos(revolute_angle)+costheta*sin(revolute_angle));
          temp_point.z = Path_set_init[i].path_pointcloud.points[j].z + z_height;
        }
        Path_sub_path.path_pointcloud.points.push_back(temp_point);
      }
      double temp_path_id = Path_set[i].path_pointcloud.channels[0].values[0] + path_scale;
      Path_sub_path.path_pointcloud.channels.resize(1);
      Path_sub_path.path_pointcloud.channels[0].name = "path_id";
      Path_sub_path.path_pointcloud.channels[0].values.resize(Path_sub_path.path_pointcloud.points.size(),1);
      Path_sub_path.path_id = id_num_;
      id_num_++;
      Path_set.push_back(Path_sub_path);
    }
    z_height+=10;
    path_scale+=1000;
  }

  id_num_ = 0;
}

bool Planning::ReadAdjacentListTXT(string Adjacent_list_folder,string Adjacent_list_name,vector<MapToPath> &map_to_path) {
  map_to_path.clear();

  std::ifstream node_file;

  std::string file_name_node;
	file_name_node = Adjacent_list_folder + Adjacent_list_name + ".txt";

  node_file.open(file_name_node);

  if(!node_file.is_open()) {
  	cout<<"Could not find Node File"<<endl;
  	cout<<"File Not Exist At: "<<file_name_node<<endl;
  	return false;
  }

  string str;
  while (std::getline(node_file, str)) {
    MapToPath sub_map_to_path;
		std::stringstream ss(str);

    bool map_enable = true;
    int sub_path_id;

    while(!ss.eof()) {
      if(map_enable) ss >> sub_map_to_path.map_data_num;
      else {
        if(ss >> sub_path_id)
          sub_map_to_path.path_id.push_back(sub_path_id);
      }
      map_enable = false;
    }

    map_to_path.push_back(sub_map_to_path);
	}
	node_file.close();

  // for(int m = 0; m < map_to_path.size(); m++) {
  //   cout << "------------------------------" << endl;
  //   cout << "map_num : " << map_to_path[m].map_data_num << endl;
  //   cout << "path_id : ";
  //   for(int n = 0; n < map_to_path[m].path_id.size(); n++) {
  //     cout <<  map_to_path[m].path_id[n] << " , ";
  //   }
  //   cout << endl;
  // }

	return true;
}

void Planning::ComputeSafePath(nav_msgs::OccupancyGrid Cost_map,vector<PathGroup> Path_set,vector<MapToPath> map_to_path) {
  path_safe_set_.clear();
  path_set_init_.clear();
  path_set_init_ = Path_set;

  vector<int> temp_path_id;
  for(int i = 0; i < Cost_map.data.size(); i++) {
    if(Cost_map.data[i] < 10) continue;
    int single_path_id;
    for(int j = 0; j < map_to_path[i].path_id.size(); j++) {
      single_path_id = map_to_path[i].path_id[j];
      path_set_init_[single_path_id].path_pointcloud.channels[0].values.assign(path_set_init_[single_path_id].path_pointcloud.points.size(),-1);
      temp_path_id.push_back(single_path_id);
    } 
  }

  for(int ii = 0; ii < path_set_init_.size(); ii++) {
    PathGroup temp_path_group;
    if(path_set_init_[ii].path_pointcloud.channels[0].values.front() != -1) {
      double path_group_id = path_set_init_[ii].path_id / (spline_2nd_level_*spline_3rd_level_);
      temp_path_group.path_id = path_set_init_[ii].path_id;
      temp_path_group.path_pointcloud = path_set_init_[ii].path_pointcloud;
      path_safe_set_.push_back(temp_path_group);
    }
  }
  cout << "path_safe_set_.size() : " << path_safe_set_.size() << endl;
  return;
}

bool Planning::SelectBestPath(geometry_msgs::Point32 Goal) {

  path_best_.points.clear();
  sub_1_path_best_.points.clear();
  sub_2_path_best_.points.clear();
  
  double goal_weight = 3;
  static int last_index = -1;
  int optimal_index;
  bool isLastEnable = false;

  int path_group_index;
  vector<int> sub_path_index;
  vector<double> path_cost(spline_array_num_,0);
  vector<int> sub_path_cost(spline_2nd_level_,0);
  if(path_safe_set_.empty()) return false;

  // compute path group cost, choose max_feasible_pro fist group path
  for(int i = 0; i < path_safe_set_.size(); i++) {
    int path_1st_group_index = path_safe_set_[i].path_id / (spline_2nd_level_*spline_3rd_level_);
    geometry_msgs::Point32 candidate_point = path_safe_set_[i].path_pointcloud.points.back();
    double path_goal_distance = hypot((candidate_point.y - Goal.y),(candidate_point.x - Goal.x));
    path_cost[path_1st_group_index] += 1.0 / pow(path_goal_distance,goal_weight);
  }


  double max_feasible_prob = -1;
  for(int j = 0; j < path_cost.size(); j++) {
    if(path_cost[j] > max_feasible_prob) {
      max_feasible_prob = path_cost[j];
      path_group_index = j;
    }
  }

  cout << "path_group_index : "  << path_group_index << endl;
  

  // compute sub path cost, choose sub_max_feasible_pro second group path
  for(int k = 0; k < path_safe_set_.size(); k++) {
    if(path_safe_set_[k].path_id / (spline_2nd_level_*spline_3rd_level_) == path_group_index) {
      int check_id = path_safe_set_[k].path_id % (spline_2nd_level_*spline_3rd_level_);
      int check_index = check_id / spline_3rd_level_;
      sub_path_cost[check_index] += 1;

    }
  }

  int sub_max_feasible_prob = INT_MIN;
  for(int ii = 0; ii < sub_path_cost.size(); ii++) {
    cout << ii << " : "<< sub_path_cost[ii] << endl;
    if(sub_path_cost[ii] > sub_max_feasible_prob) {
      cout << ii << "  **********" << endl;
      optimal_index = path_group_index*spline_2nd_level_ + ii;
      sub_max_feasible_prob = sub_path_cost[ii];
    }
  }

  for (int jj = 0; jj < sub_path_cost.size(); jj++) {
    if(sub_path_cost[jj] == sub_max_feasible_prob) {
      sub_path_index.push_back(jj);
      if(path_group_index*spline_2nd_level_+jj == last_index) isLastEnable = true;
    }
  }

  int sub_max_feasible_prob_index = sub_path_index.size()/2;
  optimal_index = path_group_index * spline_2nd_level_ + sub_path_index[sub_max_feasible_prob_index];

  // 最优第二级子路径
  bool enable_push_back = true;
  for(int k = 0; k < path_safe_set_.size(); k++) {
    if(path_safe_set_[k].path_id / spline_3rd_level_ == optimal_index) {
      for(int m = 0; m < path_safe_set_[k].path_pointcloud.points.size(); m++) {
        geometry_msgs::Point32 temp_point;
        temp_point = path_safe_set_[k].path_pointcloud.points[m];
        temp_point.z = 400;
        sub_1_path_best_.points.push_back(temp_point);
        if(enable_push_back) {
          sub_2_path_best_.points.push_back(temp_point);
        }
      }
      enable_push_back = false;
    }
  }

  cout << "optimal_index : " << optimal_index << endl;
  last_index = optimal_index;

  return true;
}

bool Planning::CheckNodeRepeat(int check_id, vector<int> id_group) {
  if(id_group.empty()) return false;
  for(int i = 0; i < id_group.size(); ++i) {
    if(check_id == id_group[i]) return true; 
  }
  return false;
}

sensor_msgs::PointCloud Planning::ConvertVectortoPointcloud(vector<PathGroup> Input) {
  sensor_msgs::PointCloud temp_pointcloud;
  if(Input.empty()) return temp_pointcloud;
  temp_pointcloud.channels.resize(1);
  temp_pointcloud.channels[0].name = "path_id";

  temp_pointcloud.header.frame_id = "/base_link";
  temp_pointcloud.header.stamp = ros::Time::now();
  for(int j = 0; j < Input.size(); j++) {
    for(int i = 0; i < Input[j].path_pointcloud.points.size(); i++) {
      temp_pointcloud.points.push_back(Input[j].path_pointcloud.points[i]);
      temp_pointcloud.channels[0].values.push_back(Input[j].path_pointcloud.channels[0].values[i]);
    }
  }
}

void Planning::ExpandCostmap(nav_msgs::OccupancyGrid &Grid,int expand_size) {
  if(expand_size <= 0) return;
  for(int index = 0; index < Grid.data.size(); index++) {
    if(Grid.data[index] == 50) {
      int col_num = index / Grid.info.width; //width 是x方向的长度
      int row_num = index % Grid.info.width;
      for(int i = col_num - expand_size; i <= col_num + expand_size; i++) {
        if(i < 0 || i > Grid.info.height - 1) continue;
        for(int j = row_num - expand_size; j <= row_num + expand_size; j++) {
          if(j < 0 || j > Grid.info.width - 1) continue;
          if(Grid.data[i * Grid.info.width + j] != 50) {
            Grid.data[i * Grid.info.width + j] = 55;
          }
        }
      }

    } 
  }
}


