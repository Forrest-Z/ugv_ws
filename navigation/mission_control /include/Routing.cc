#include <Routing.h>


/******************************************************************************************
 ****                                 Generate Routing                                 ****
 ******************************************************************************************
 * Main function in Routing class, call for generate global route                         *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Goal_in_map    - Route goal 2D position (x,y)                                      *
 *     Vehicle_in_map - Vehicle 2D Pose (x,y,yaw) (.z represents yaw)                     *
 *     Map_folder     - Folder directory for map, map yaml file and node file             *
 *     Map_number     - Map ID for map changing purpose                                   *
 * Output:                                                                                *
 *     path_pointcloud_ - A global variable, call path_pointcloud() function to           *
 *                        obtain route                                                    *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 ******************************************************************************************/
void Routing::RoutingAnalyze(geometry_msgs::Point32& Goal_in_map,geometry_msgs::Point32 Vehicle_in_map,string Map_folder,int Map_number) {
	CleanAllState();
	if(!ReadNodeTXT(Map_folder,Map_number)) return;
	if(!ReadYamlFile(Map_folder,Map_number)) return;
	int start_id = FindPointId(Vehicle_in_map);
	int end_id = FindPointId(Goal_in_map);

	// cout << "From     " << start_id << " to " << end_id << endl;
	// cout << "Robot at " << Vehicle_in_map.x << "," << Vehicle_in_map.y << endl;
	// cout << "Goal  at " << Goal_in_map.x << "," << Goal_in_map.y << endl;
	if(start_id == end_id) {
		// cout << "Global Plan Failed" << endl;
		// cout << "ID Duplicate" << endl;
		return;
	}
	if(!ComputePath(start_id,end_id)) {
		cout << "Global Plan Failed" << endl;
		cout << "From     " << start_id << " to " << end_id << endl;
		cout << "Robot at " << Vehicle_in_map.x << "," << Vehicle_in_map.y << endl;
		cout << "Goal  at " << Goal_in_map.x << "," << Goal_in_map.y << endl;
		return;
	}
	SetPathtoPointcloud(Goal_in_map);
}


/******************************************************************************************
 ****                                 Compute Routing                                  ****
 ******************************************************************************************
 * Modified A-star algorithm, Heuristic cost added routing cost from map information      *
 * Search and compute cost on map pixel frame                                             *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Start_Id - Search Start Point Id from node file                                    *
 *     End_id   - Search Goal Point Id from node file                                     *
 * Output:                                                                                *
 *     Return   - return false when no valid path                                         *
 *     path_    - A global variable vector stored path IDs in order                       *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 *     vector<MapGraph> node_info_ - defined structure for each node informations         *
 ******************************************************************************************/
bool Routing::ComputePath(int Start_Id,int End_id) {
	if(!checkIndexVaild(Start_Id,node_info_)) {
		cout<<"Invalid Start Point"<<endl;
		return false;
	}
	if(!checkIndexVaild(End_id,node_info_)) {
		cout<<"Invalid End Point"<<endl;
		return false;
	}

	// priority queue for store a-star path
	std::priority_queue<MapGraph,vector<MapGraph>,NodeCompare> path_astar_que;

	vector<int> path;
	queue<int> to_visit;
	vector<int> neighbor;
	vector<int> visited(node_info_.size()+1,-1);
	vector<double> costs(node_info_.size()+1,-1);

	visited[Start_Id] = 0;
	costs[Start_Id] = 0;
	int current_id = Start_Id;

	findNeighbor(current_id,node_info_,neighbor);

	// explore and compare to find minimum cost at current position until reach the goal
	while(!isGoalReached_) {
		for (int i = 0; i < neighbor.size() ; i++) {
			int next_id = neighbor[i];
			double new_cost = costs[current_id] + computeHeuristic(current_id,next_id,node_info_);
			if(visited[next_id]==-1 || new_cost<costs[next_id]) {
				visited[next_id] = current_id;
				costs[next_id] = new_cost;
				to_visit.push(next_id);
			}
		}
		if(to_visit.empty()) {
			isGoalReached_ = true;
			break;
		}
		current_id = to_visit.front();
		to_visit.pop();
		findNeighbor(current_id,node_info_,neighbor);
	}

	// Check when queue is valid, push IDs into global vector.
	int next_point = End_id; 
	while(!isPathReady_) {
		if (next_point == 0) {
			isPathReady_ = true;
			break;
		}
		if(next_point == -1) return false;
		path.push_back(next_point);
		next_point = visited[next_point];
	}
	path_ = path;
	for(int i = 0; i < path.size(); i++) {
		cout << path[i] << " "; 
	}
	cout << endl;

	return true;
}


void Routing::CleanAllState() {
	isPathReady_   = false;
	isGoalReached_ = false;
	node_info_.clear();
	path_pointcloud_.points.clear();
	path_.clear();
}


/******************************************************************************************
 ****                       Convert ID vector to Pointcloud Path                       ****
 ******************************************************************************************
 * Convert vector of node ID into Pointcloud Path, direct connect two node when the angle * 
 * of path curve is smaller than threshold, otherwise connect in Bezier Curves            *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     path_            - A global variable vector stored path IDs in order               *
 * Output:                                                                                *
 *     path_pointcloud_ - A global variable, call path_pointcloud() function to           *
 *                        obtain route                                                    *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     double points_gap           - points distance(m) between two node in goal route    *
 *     double curve_threshold      - threshold of radian to generate curve instead line   *
 *     double curve_ratio          - points density ration between curve and line         *
 *   Global:                                                                              *
 *     vector<MapGraph> node_info_ - defined structure for each node informations         *
 *     YamlInfo map_info_          - defined structure for descried map image to map data *
 ******************************************************************************************/
void Routing::SetPathtoPointcloud(geometry_msgs::Point32 Goal) {
	int node_size = path_.size();
  double points_gap = 3;
  double curve_threshold = 2;
  double curve_ratio = 2;
  geometry_msgs::Point32 last_point;

  geometry_msgs::Point32 point_1st;
  geometry_msgs::Point32 point_2nd;

  int path_order = 0;
  int node_index = path_[path_order] - 1;
  point_1st.x = (node_info_[node_index].position.y / map_info_.ratio) * map_info_.resolution 
    + map_info_.origin.x;
  point_1st.y = (map_info_.height - (node_info_[node_index].position.x / map_info_.ratio)) * map_info_.resolution 
    + map_info_.origin.y;
  point_1st.z = node_info_[node_index].position.z;

  path_order++;
  node_index = path_[path_order] - 1;

  point_2nd.x = (node_info_[node_index].position.y / map_info_.ratio) * map_info_.resolution 
    + map_info_.origin.x;
  point_2nd.y = (map_info_.height - (node_info_[node_index].position.x / map_info_.ratio)) * map_info_.resolution 
    + map_info_.origin.y;
  point_2nd.z = node_info_[node_index].position.z;



  double last_closet_dis = hypot((Goal.x-point_2nd.x),(Goal.y-point_2nd.y));
  double closet_dis = hypot((point_1st.x-Goal.x),(point_1st.y-Goal.y));
	double two_point_dis = hypot((point_1st.x-point_2nd.x),(point_1st.y-point_2nd.y));

	double cos_angle_2nd = (pow(last_closet_dis,2) + pow(two_point_dis,2) - pow(closet_dis,2))/(2 * last_closet_dis * two_point_dis);
	double cos_angle_1st = (pow(closet_dis,2) + pow(two_point_dis,2) - pow(last_closet_dis,2))/(2 * closet_dis * two_point_dis);

	double angle_2nd = acos(cos_angle_2nd);
	double angle_1st = acos(cos_angle_1st);

	sensor_msgs::PointCloud pointcloud;
  	pointcloud.header.frame_id = "/map";
	for (int i = 0; i < node_size; i++) {
		geometry_msgs::Point32 point;
		path_order = node_size - i - 1;
		node_index = path_[path_order] -1;
		point.x = (node_info_[node_index].position.y / map_info_.ratio) * map_info_.resolution 
		+ map_info_.origin.x;
		point.y = (map_info_.height - (node_info_[node_index].position.x / map_info_.ratio)) * map_info_.resolution 
		+ map_info_.origin.y;
		point.z = node_info_[node_index].position.z;

    if(i > 0) {
		  double diff_x = point.x - last_point.x;
		  double diff_y = point.y - last_point.y;
	    double distance = hypot(diff_x,diff_y);
	    int points_number = distance / points_gap;
	    double points_angle = atan2(diff_y,diff_x);

	    for (int j = 1; j < points_number; ++j) {
	      geometry_msgs::Point32 point_mid;
	      point_mid.x = last_point.x + j * points_gap * cos(points_angle);
	      point_mid.y = last_point.y + j * points_gap * sin(points_angle);
	      pointcloud.points.push_back(point_mid);
	    }
    } 
    last_point = point;
  	pointcloud.points.push_back(point);
  }


	if(angle_2nd < 1.58 && angle_1st < 1.58) {
		pointcloud.points.back() = Goal;
	} else {
		pointcloud.points.push_back(Goal);
	}

  for (int i = 0; i < pointcloud.points.size() - 2; ++i) {
  	geometry_msgs::Point32 point_1 = pointcloud.points[i];
  	geometry_msgs::Point32 point_2 = pointcloud.points[i+1];
  	geometry_msgs::Point32 point_3 = pointcloud.points[i+2];

    double distance_12 = hypot((point_1.x - point_2.x),(point_1.y - point_2.y));
    double distance_23 = hypot((point_2.x - point_3.x),(point_2.y - point_3.y));	
    double distance_13 = hypot((point_1.x - point_3.x),(point_1.y - point_3.y));	
    double cosin_2_numerator = pow(distance_12,2) + pow(distance_23,2) - pow(distance_13,2);
    double cosin_2_denominator = 2 * distance_12 * distance_23;
    double cosin_2 = cosin_2_numerator/cosin_2_denominator;
    double radian_2 = acos(cosin_2);

    double point_gap_curve = points_gap/curve_ratio;
	int points_number_curve = distance_13/point_gap_curve;

    if(radian_2 < curve_threshold) {
  		sensor_msgs::PointCloud pointcloud_curve;
			pointcloud_curve.header.frame_id = "/map";
	    for (int j = 0; j <= points_number_curve; ++j) {
	      geometry_msgs::Point32 point_mid;
	      double tt = static_cast<double>(j)/static_cast<double>(points_number_curve);

	      point_mid.x = (1-tt)*((1-tt)*point_1.x+tt*point_2.x) + tt*((1-tt)*point_2.x + tt*point_3.x);
	      point_mid.y = (1-tt)*((1-tt)*point_1.y+tt*point_2.y) + tt*((1-tt)*point_2.y + tt*point_3.y);
	      pointcloud_curve.points.push_back(point_mid);

    	}
    	pointcloud.points.erase(pointcloud.points.begin()+i,pointcloud.points.begin()+i+3);
    	pointcloud.points.insert(pointcloud.points.begin()+i,pointcloud_curve.points.begin(),pointcloud_curve.points.end());
    	i+=pointcloud_curve.points.size();
    	continue;
    }
  }

  path_pointcloud_ = pointcloud;
}


/******************************************************************************************
 ****                               Modify Route Point                                 ****
 ******************************************************************************************
 * Modify original route point based on map static obstacles                              *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Input    - original route point                                                    *
 *     Obstacle - fusion obstacle include map and sensor data                             *
 * Output:                                                                                *
 *     output   - modified route point                                                    *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 ******************************************************************************************/
void Routing::ModifyRoutePoint(geometry_msgs::Point32 Input,sensor_msgs::PointCloud& Output,sensor_msgs::PointCloud Obstacle) {
	Output.header.frame_id = Obstacle.header.frame_id;
	Output.header.stamp = Obstacle.header.stamp;
	Output.points.clear();

	double left_min_distance_to_route = DBL_MAX;
	double right_min_distance_to_route = DBL_MAX;  
	int left_min_obstacle_index = -1;
	int right_min_obstacle_index = -1;


	double left_max_distance_to_route = -1;
	double right_max_distance_to_route = -1;
	int left_max_obstacle_index = -1;
	int right_max_obstacle_index = -1;

	double arc_point_gap = 1;

	for (int i = 0; i < Obstacle.points.size(); ++i) {
		if(Obstacle.points[i].z < 5) continue;
		if(hypot(Obstacle.points[i].x,Obstacle.points[i].y) > 8) continue;
		double diff_x = fabs(Obstacle.points[i].x - Input.x);
		double diff_y = fabs(Obstacle.points[i].y - Input.y);
		double temp_distance = hypot(diff_x,diff_y);

		if(Obstacle.points[i].x > 0 && Obstacle.points[i].y > 0) {
			if(temp_distance < left_min_distance_to_route) {
				left_min_distance_to_route = temp_distance;
				left_min_obstacle_index = i;
			} 
		} 
		else if(Obstacle.points[i].x < 0 && Obstacle.points[i].y > 0) {
			if(temp_distance > left_max_distance_to_route) {
				left_max_distance_to_route = temp_distance;
				left_max_obstacle_index = i;
			}		
		}
		else if(Obstacle.points[i].x > 0 && Obstacle.points[i].y < 0) {
			if(temp_distance < right_min_distance_to_route) {
				right_min_distance_to_route = temp_distance;
				right_min_obstacle_index = i;
			} 	
		}
		else if(Obstacle.points[i].x < 0 && Obstacle.points[i].y < 0) {
			if(temp_distance > right_max_distance_to_route) {
				right_max_distance_to_route = temp_distance;
				right_max_obstacle_index = i;
			}	
		}
	}

	if(left_min_obstacle_index != -1) {
		Obstacle.points[left_min_obstacle_index].z = 10;
		Output.points.push_back(Obstacle.points[left_min_obstacle_index]);
	}
	if(right_min_obstacle_index != -1) {
		Obstacle.points[right_min_obstacle_index].z = 9;
		Output.points.push_back(Obstacle.points[right_min_obstacle_index]);
	}
	if(left_max_obstacle_index != -1) {
		Obstacle.points[left_max_obstacle_index].z = 8;
		Output.points.push_back(Obstacle.points[left_max_obstacle_index]);
	}
	if(right_max_obstacle_index != -1) {
		Obstacle.points[right_max_obstacle_index].z = 7;
		Output.points.push_back(Obstacle.points[right_max_obstacle_index]);
	}

	if(left_min_obstacle_index != -1 && right_min_obstacle_index != -1) {
		geometry_msgs::Point32 temp_point_2;
		temp_point_2.x =(Obstacle.points[left_min_obstacle_index].x + Obstacle.points[right_min_obstacle_index].x)/2;
		temp_point_2.y =(Obstacle.points[left_min_obstacle_index].y + Obstacle.points[right_min_obstacle_index].y)/2;
		temp_point_2.z = 1;
		Output.points.push_back(temp_point_2);

		geometry_msgs::Point32 temp_point_3;
		temp_point_3.x =(Obstacle.points[left_min_obstacle_index].x + Obstacle.points[right_min_obstacle_index].x + Input.x)/3;
		temp_point_3.y =(Obstacle.points[left_min_obstacle_index].y + Obstacle.points[right_min_obstacle_index].y + Input.y)/3;
		temp_point_3.z = 2;
		Output.points.push_back(temp_point_3);

		double mid_diff_x = fabs(temp_point_2.x - temp_point_3.x);
		double mid_diff_y = fabs(temp_point_2.y - temp_point_3.y);
		double mid_distance = hypot(mid_diff_x,mid_diff_y);

		double point_diff_x = fabs(Input.x - temp_point_3.x);
		double point_diff_y = fabs(Input.y - temp_point_3.y);
		double point_distance = hypot(point_diff_x,point_diff_y);
		if(mid_distance < 3 && point_distance < 2) {
			temp_point_3.z = -1;
			Output.points.push_back(temp_point_3);
		}
	}

	return;
}


/******************************************************************************************
 ****                                  Read Node File                                  ****
 ******************************************************************************************
 * Read text file save as vector of nodes information                                     *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Map_folder - Folder directory for map, map yaml file and node file                 *
 *     Map_number - Map ID for map changing purpose                                       *
 * Output:                                                                                *
 *     Return     - return false when file not exist                                      *
 *     node_info_ - A global variable stored nodes informations                           *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     vector<int> read_neighbor - create buffer for potential node neighbors             *
 *   Global:                                                                              *
 ******************************************************************************************/
bool Routing::ReadNodeTXT(string Map_folder,int Map_number) {
	MapGraph graph;
  std::ifstream node_file;

  std::string file_name_node;
	file_name_node = Map_folder + std::to_string(Map_number) + ".txt";

  node_file.open(file_name_node);
  if(!node_file.is_open()) {
  	cout<<"Could not find Node File"<<endl;
  	cout<<"File Not Exist At: "<<file_name_node<<endl;
  	return false;
  }

  int linenum;
  string str;
  while (std::getline(node_file, str)) {
		std::stringstream ss(str);
		int xxx,yyy,zzz;
		vector<int> read_neighbor(8.0);
		ss >> linenum >> xxx >> yyy >> zzz >> read_neighbor[0] >> read_neighbor[1] >> read_neighbor[2] >> read_neighbor[3] >> read_neighbor[4] >> read_neighbor[5] >> read_neighbor[6] >> read_neighbor[7];
		graph.id = linenum;
		graph.position.x = xxx;
		graph.position.y = yyy;
		graph.position.z = zzz;
		graph.neighbor = read_neighbor;
		node_info_.push_back(graph);
	}
	node_file.close();
	return true;
}

/******************************************************************************************
 ****                                  Read Yaml File                                  ****
 ******************************************************************************************
 * Read text file save as map's yaml                                                      *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Map_folder - Folder directory for map, map yaml file and node file                 *
 *     Map_number - Map ID for map changing purpose                                       *
 * Output:                                                                                *
 *     Return     - return false when file not exist                                      *
 *     map_info_  - A global variable stored map informations                             *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *     int information_required_size - preset information categorize number               *
 *   Global:                                                                              *
 ******************************************************************************************/
bool Routing::ReadYamlFile(string Map_folder,int Map_number){
  std::ifstream yaml_file;
  std::string file_name_node = Map_folder + std::to_string(Map_number) + "_info.txt";
  yaml_file.open(file_name_node);
  if(!yaml_file.is_open()) {
  	cout<<"Could not find Yaml File"<<endl;
  	cout<<"File Not Exist: "<<file_name_node<<endl;
  	return false;
  }
  int information_required_size = 5;
  string str;
  int info_linenum = 0;
  while (std::getline(yaml_file, str)) {
		std::stringstream ss(str);
		string yaml_info;
		ss >> yaml_info;

		if(yaml_info == "resolution") {	
			ss >> map_info_.resolution;
			info_linenum++;
		}
		else if(yaml_info == "origin") {
			ss >> map_info_.origin.x >> map_info_.origin.y;
			info_linenum++;
		}
		else if(yaml_info == "width") {
			ss >> map_info_.width;
			info_linenum++;
		}
		else if(yaml_info == "height") {
			ss >> map_info_.height;
			info_linenum++;
		}
		else if(yaml_info == "ratio") {
			ss >> map_info_.ratio;
			info_linenum++;
		}
	}

	yaml_file.close(); 

	if(info_linenum == information_required_size) {
		return true;
	} else {
		return false;
	}
}

/******************************************************************************************
 ****                                 Search Point ID                                  ****
 ******************************************************************************************
 * Read text file save as map's yaml                                                      *
 * Explanation : TODO                                                                     *
 * Input:                                                                                 *
 *     Input  - point position in map frame                                               *
 * Output:                                                                                *
 *     output - point ID in node file                                                     *
 * Variables:                                                                             *
 *   Local :                                                                              *
 *   Global:                                                                              *
 ******************************************************************************************/
int Routing::FindPointId(geometry_msgs::Point32 Input) {
	double closet_dis = DBL_MAX;
	int closet_id = -1;
	int output = -1;
	geometry_msgs::Point32 point_in_pixel = ConvertMapToPixel(Input);

	for (int i = 0; i < node_info_.size(); i++) {
		double current_dis = hypot(fabs(node_info_[i].position.x-point_in_pixel.x),fabs(node_info_[i].position.y-point_in_pixel.y));
		if(current_dis <= closet_dis) {
			closet_dis = current_dis;
			closet_id = i;
		}
	}

	output = closet_id + 1;
	return output;
}