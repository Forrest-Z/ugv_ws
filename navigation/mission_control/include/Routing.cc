#include <Routing.h>

void Routing::RoutingAnalyze(geometry_msgs::Point32& Goal_in_map,geometry_msgs::Point32 Vehicle_in_map,string Map_folder,int Map_number) {

	CleanAllState();

	if(!ReadNodeTXT(Map_folder,Map_number)) return;
	if(!ReadYamlFile(Map_folder,Map_number)) return;


	int start_id = FindPointId(Vehicle_in_map);
	int end_id = FindPointId(Goal_in_map);

	if(!ComputePath(start_id,end_id)) {
		cout << "Global Plan Failded" << endl;
		cout << "From     " << start_id << " to " << end_id << endl;
		cout << "Robot at " << Vehicle_in_map.x << "," << Vehicle_in_map.y << endl;
		cout << "Goal  at " << Goal_in_map.x << "," << Goal_in_map.y << endl;
		return;
	}

	SetPathtoPointcloud();
	// Goal_in_map = *(path_pointcloud_.points.end()-1);
}

bool Routing::ComputePath(int Start_Id,int End_id) {
	// Modified A-star algorithm, Heuristic cost added routing cost from map information. 
	if(!checkIndexVaild(Start_Id,node_info_)) {
		cout<<"Invaild Start Point"<<endl;
		return false;
	}
	if(!checkIndexVaild(End_id,node_info_)) {
		cout<<"Invaild End Point"<<endl;
		return false;
	}

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

	return true;
}

void Routing::CleanAllState() {
	isPathReady_   = false;
	isGoalReached_ = false;
	node_info_.clear();
	path_pointcloud_.points.clear();
	path_.clear();
}

void Routing::SetPathtoPointcloud() {
	int node_size = path_.size();
  double points_gap = 5;
  geometry_msgs::Point32 last_point;

	sensor_msgs::PointCloud pointcloud;
  pointcloud.header.frame_id = "/map";
	for (int i = 0; i < node_size; i++) {
    geometry_msgs::Point32 point;
    int path_order = node_size - i - 1;
    int node_index = path_[path_order] -1;
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
	      point_mid.z = last_point.z;
	      pointcloud.points.push_back(point_mid);
	    }
    } 
    last_point = point;
  	pointcloud.points.push_back(point);
  }

  path_pointcloud_ = pointcloud;
}

bool Routing::ReadNodeTXT(string Map_folder,int Map_number) {
	// Read text file for node information.
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

  static int linenum = 1;
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


bool Routing::ReadYamlFile(string Map_folder,int Map_number){
	// Read text file as map's yaml file.
  std::ifstream yaml_file;
  std::string file_name_node = Map_folder + std::to_string(Map_number) + "_info.txt";
  yaml_file.open(file_name_node);
  if(!yaml_file.is_open()) {
  	cout<<"Could not find Yaml File"<<endl;
  	cout<<"File Not Exist: "<<file_name_node<<endl;
  	return false;
  }

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

	if(info_linenum == 5) {
		return true;
	} else {
		return false;
	}
}


int Routing::FindPointId(geometry_msgs::Point32 Input) {
	double closet_dis = DBL_MAX;
	int output = -1;
	geometry_msgs::Point32 point_in_pixel = ConvertMapToPixel(Input);

	for (int i = 0; i < node_info_.size(); i++) {
		double current_dis = hypot( abs(node_info_[i].position.x-point_in_pixel.x),abs(node_info_[i].position.y-point_in_pixel.y) );
		if(current_dis <= closet_dis) {
			closet_dis = current_dis;
			output = i+1;
		}
	}
	return output;
}