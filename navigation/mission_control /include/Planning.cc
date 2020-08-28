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
  vector<sensor_msgs::PointCloud> joints_set;
  vector<sensor_msgs::PointCloud> path_set;
  vector<vector<sensor_msgs::PointCloud>> path_set_2d;
  vector<vector<sensor_msgs::PointCloud>> safe_set_2d;

  mtx_radius_.lock();
  path_window_radius_ = Path_Radius;
  if(path_window_radius_ <= 0) return false;
  if(path_window_radius_ > map_window_radius_) return false;

  if(!UpdateCostmap(Obstacle)) return false;
  path_result = CheckIdealPath(Goal);

  if(!path_result){
    GenerateSplinesJoints(joints_set);
    GenerateSplinesPath(path_set,joints_set,path_set_2d);
    ComputeSafePath(path_set_2d,safe_set_2d,path_safe_set_,costmap_local_);
    path_result = SelectBestPath(safe_set_2d,Goal);
  }
  path_all_set_ = path_set;
  mtx_radius_.unlock();
 
  if(!path_result) {
    sensor_msgs::PointCloud path_temp;
    geometry_msgs::Point32 point_temp;
    path_temp.header.frame_id = "/base_link";
    path_temp.header.stamp = ros::Time::now();
    path_temp.points.push_back(point_temp);
    path_best_ = path_temp;
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
void Planning::GenerateSplinesJoints(vector<sensor_msgs::PointCloud>& Output) {
  Output.clear();
  const int temp_splines_joints_num = 3;

  double shrink_scale = temp_splines_joints_num * 2;
  double radius_unit = path_window_radius_ / temp_splines_joints_num;

  vector<int> level_nums = {5,5,3}; //{9,9,7};
  vector<double> level_unit(temp_splines_joints_num,0);
  vector<sensor_msgs::PointCloud> temp_vector(level_nums[0]);

  level_unit[0] = path_swap_range_ / double(level_nums[0] - 1);
  for (int i = 1; i < level_unit.size(); ++i) {
    level_unit[i] = level_unit[i-1] * double(level_nums[i-1] - 1) / (shrink_scale * double(level_nums[i] - 1));
  }

  sensor_msgs::PointCloud temp_level;
  temp_level.header.frame_id = "/base_link";
  temp_level.header.stamp = ros::Time::now();
  geometry_msgs::Point32 temp_joint;

  for (int i = 0; i < temp_vector.size(); ++i) {
    double level_0_angle = -(path_swap_range_/2) + i*level_unit[0];
    temp_joint.x = radius_unit * 0.5 * cos(level_0_angle);
    temp_joint.y = radius_unit * 0.5 * sin(level_0_angle); // 由radius_unit*1 -> radius_unit*0.5, 缩短第一个节点距离
    temp_joint.z = i+1;
    temp_level.points.push_back(temp_joint);
    for (int j = 0; j < level_nums[1]; ++j) {
      double level_1_angle = level_0_angle - ((level_nums[1] - 1) * level_unit[1]/2) + j * level_unit[1];
      temp_joint.x = radius_unit * 2 * cos(level_1_angle);
      temp_joint.y = radius_unit * 2 * sin(level_1_angle);
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
}


/******************************************************************************************
 ****                                 Check Path Safety                                ****
 ******************************************************************************************
 * Iteration 2d vector of path to find non collision paths                                *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Input_2d  - 2d vector of all candidate path                                        *
 *     Costmap   - the costmap in local frame                                             *
 * Output:                                                                                *
 *     Output    - vector of non collision path                                           *
 *     Output_2d - 2d vector of non collision path                                        *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 *     const int ADDITION_SPLINE - addition grid when search obstacle around spline       *
 ******************************************************************************************/
void Planning::ComputeSafePath(vector<vector<sensor_msgs::PointCloud>> Input_2d,vector<vector<sensor_msgs::PointCloud>>& Output_2d,vector<sensor_msgs::PointCloud>& Output,nav_msgs::OccupancyGrid Costmap) {
  Output.clear();
  Output_2d.resize(Input_2d.size());
  for (int i = 0; i < Input_2d.size(); ++i) {
    for (int j = 0; j < Input_2d[i].size(); ++j) {
      if(!DetectObstcaleGrid(Input_2d[i][j],Costmap,ADDITION_SPLINE)) {
        continue;
      }
      Output_2d[i].push_back(Input_2d[i][j]);
      Output.push_back(Input_2d[i][j]);
    }
  }
} 


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
		    Astar_costmap_array_[ptr.x][ptr.y] = obstacle_occupancy;
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
  RRTExpandCostmap(RRT_costmap_local_);
  
  for(int search_times = 0;search_times < RRT_search_plan_num_;search_times++) {
    RRTInitialRoot(start);

    if(BuildRRT(goal)) {
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

bool Planning::BuildRRT(geometry_msgs::Point32 goal) {
  for(int i = 0; i < RRT_iteration_; i++){
    RRTProbSample(goal);
    for(int j = 0; j < RRT_current_sampling_.size(); j++){
      if(RRTExtendNearestNode(RRT_current_sampling_[j],goal))
        return true;
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
    for(int i = 0;i < RRT_sample_num_;i++) {
        RRTPointCell p;
        //这里我们可以将撒的点直接设在终点，也可以在终点附近有一个高斯分布
        p.x = goal.x;// + (RRT_rd_() % 200 - 100) * 0.2;
        p.y = goal.y;// + (RRT_rd_() % 200 - 100) * 0.2;
        RRT_current_sampling_.push_back(p);
    }
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
