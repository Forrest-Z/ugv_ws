#include <Routing.h>

void Routing::routingAnalyze(geometry_msgs::Point32 goal_in_map,string map_folder,int map_number) {
	// Main executer
	vector<MapGraph> point_index;

	if(!readNodeTXT(point_index,map_folder,map_number)) return;
	if(!readYamlFile(map_folder,map_number)) return;

	tf::StampedTransform transform;
	geometry_msgs::Point32 car_in_map;

	while(true){
		try {
	    listener.lookupTransform("/map", "/base_link",  
	                             ros::Time(0), transform);
	  }
	  catch (tf::TransformException ex) {
	    continue;
	  }
	  break;
	}

	isPathReady_ = false;
	isGoalReached_ =false;

	static vector<int> path_astar; 
	geometry_msgs::Point32 goal_in_piexl;
	geometry_msgs::Point32 car_in_piexl;
	int start_id = 1;
	int end_id = 10;
	map_ = point_index;

  car_in_map.x = transform.getOrigin().x();
  car_in_map.y = transform.getOrigin().y();

	mapToPixel(goal_in_map,goal_in_piexl);
	findPointId(point_index,goal_in_piexl,end_id);

	mapToPixel(car_in_map,car_in_piexl);
	findPointId(point_index,car_in_piexl,start_id);

	if(!pathPlanner(start_id,end_id,point_index,path_astar)) {
		cout<<"Global Plan Failded"<<endl;
	}
}

void Routing::getPath(std::vector<MapGraph>& Map,std::vector<int>& Path,YamlInfo& Map_info){
	// Read data from outside class and store in private variables. 
	Map = map_;
	Path = path_;
	Map_info = map_info_;
}

bool Routing::readNodeTXT(std::vector<MapGraph>& Index,string map_folder,int map_number) {
	// Read text file for node information.
	MapGraph graph;
  std::ifstream node_file;

  std::string file_name_node;
	file_name_node = map_folder + std::to_string(map_number) + ".txt";

  node_file.open(file_name_node);
  if(!node_file.is_open()) {
  	cout<<"Could not find Node File"<<endl;
  	cout<<"File Not Exist: "<<file_name_node<<endl;
  	return false;
  }

  static int linenum = 1;
  string str;
  while (std::getline(node_file, str)) {
		std::stringstream ss(str);
		int xxx,yyy;
		vector<int> read_neighbor(8.0);
		ss >> linenum >> xxx >> yyy>> read_neighbor[0] >> read_neighbor[1] >> read_neighbor[2] >> read_neighbor[3] >> read_neighbor[4] >> read_neighbor[5] >> read_neighbor[6] >> read_neighbor[7];
		graph.id = linenum;
		graph.position.x = xxx;
		graph.position.y = yyy;
		graph.neighbor = read_neighbor;
		Index.push_back(graph);
	}
	node_file.close();
	return true;
}

bool Routing::pathPlanner(int Start_Id,int End_id,
	std::vector<MapGraph> Graph,std::vector<int>& Path) {
	// Modified A-star algorithm, Heuristic cost added routing cost from map information. 
	if(!checkIndexVaild(Start_Id,Graph)) {
		cout<<"Invaild Start Point"<<endl;
		Path.clear();
		return false;
	}
	if(!checkIndexVaild(End_id,Graph)) {
		cout<<"Invaild End Point"<<endl;
		Path.clear();
		return false;
	}

	std::priority_queue<MapGraph,vector<MapGraph>,NodeCompare> path_astar_que;
	
	queue<int> to_visit;
	vector<int> neighbor;
	vector<int> visited(Graph.size()+1,-1);
	vector<double> costs(Graph.size()+1,-1);

	visited[Start_Id] = 0;
	costs[Start_Id] = 0;
	int current_id = Start_Id;

	findNeighbor(current_id,Graph,neighbor);

	while(!isGoalReached_) {
		Path.clear();
		for (int i = 0; i < neighbor.size() ; i++) {
			int next_id = neighbor[i];
			double new_cost = costs[current_id] + computeHeuristic(current_id,next_id,Graph);
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
		findNeighbor(current_id,Graph,neighbor);
	}

	int next_point = End_id; 
	while(!isPathReady_) {
		if (next_point == 0) {
			isPathReady_ = true;
			break;
		}
		Path.push_back(next_point);
		next_point = visited[next_point];
	}
	path_ = Path;
	return true;
}

double Routing::computeHeuristic(int Start_Id,int End_id,std::vector<MapGraph> Index) {
	double distances_cost = hypot(abs(Index[End_id-1].position.x-Index[Start_Id-1].position.x)
      ,abs(Index[End_id-1].position.y-Index[Start_Id-1].position.y));
   double global_cost = computeRoutingCost(Start_Id,End_id);

   return (distances_cost + global_cost);
}

double Routing::computeRoutingCost(int id_1,int id_2) {
  for (int i = 0; i < routing_info_.size(); ++i) {
    if( id_1 == routing_info_[i].id_1  || id_2 == routing_info_[i].id_1 ) {
        return routing_info_[i].cost;
    }
  }
  return 0;
}

void Routing::findNeighbor(int Id,std::vector<MapGraph> Index,std::vector<int>& Neighbor) {
	Neighbor.clear();

	for (int i = 0; i < Index[Id-1].neighbor.size(); i++)
	{
		if( Index[Id-1].neighbor[i] == 0 ) continue;
		Neighbor.push_back(Index[Id-1].neighbor[i]);
	}
}

bool Routing::readYamlFile(string map_folder,int map_number){
	// Read text file as map's yaml file.
  std::ifstream yaml_file;
  std::string file_name_node = map_folder + std::to_string(map_number) + "_info.txt";
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

void Routing::mapToPixel(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output) {
	Output.x = (map_info_.height - ((Input.y - map_info_.origin.y)/map_info_.resolution)) * map_info_.ratio;
	Output.y = ((Input.x - map_info_.origin.x)/map_info_.resolution) * map_info_.ratio;
}

void Routing::findPointId(std::vector<MapGraph>& Map,geometry_msgs::Point32 Input,int& Output) {
	double closet_dis = DBL_MAX;
	int closet_point;
	for (int i = 0; i < Map.size(); i++) {
		double current_dis = hypot( abs(Map[i].position.x-Input.x),abs(Map[i].position.y-Input.y) );
		if(current_dis<=closet_dis) {
			closet_dis = current_dis;
			closet_point = i+1;
		}
	}
	Output = closet_point;
}