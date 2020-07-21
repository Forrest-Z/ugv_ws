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

  vector<int> level_nums = {7,5,3}; //{9,9,7};
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
    temp_joint.x = radius_unit * 1 * cos(level_0_angle);
    temp_joint.y = radius_unit * 1 * sin(level_0_angle);
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
