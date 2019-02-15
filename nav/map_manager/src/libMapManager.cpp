#include "libMapManager.h"

MapManager::MapManager():pn("~")
{
  goal_sub = n.subscribe("/move_base_simple/goal",1, &MapManager::goal_callback,this);
  map_number_sub = n.subscribe("/map_number",1, &MapManager::map_number_callback,this);

  plan_pub = n.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan",1);
  plan_point_pub = n.advertise<sensor_msgs::PointCloud>("/Astar_path_point",1);

  pn.param("map_folder", map_folder_, std::string("/map"));

  sleep(1);
  Initialization();
}

MapManager::~MapManager(){
}

void MapManager::Initialization() {
	isGoalReached_ = false; isGoalSend_=false;isPathReady_=false;isReadYaml_=false;
	map_number_ = 1;
}

void MapManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  while (ros::ok()) {
  	Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }

  NaviManager test;
  test.testFunction();
}

void MapManager::Mission() {
	routingAnalyze();
}

void MapManager::routingAnalyze() {
	vector<MapGraph> point_index;
	readNodeTXT(point_index);

	saved_graph_ = point_index;
	static vector<int> path_astar; 
	geometry_msgs::Point32 goal_in_piexl;
	geometry_msgs::Point32 car_in_piexl;
	int start_id = 1;
	int end_id = 10;
	if(!readYamlFile()) return;
	if(!isGoalSend_) {
		ROS_INFO_ONCE("Waiting for Goal");
		return;
	}

	tf::StampedTransform transform;
	geometry_msgs::Point32 car_in_map;
	try {
    listener.lookupTransform("/map", "/base_link",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    return;
  }

  car_in_map.x = transform.getOrigin().x();
  car_in_map.y = transform.getOrigin().y();

	goalToPixel(goal_in_map_,goal_in_piexl);
	findPointId(saved_graph_,goal_in_piexl,end_id);

	goalToPixel(car_in_map,car_in_piexl);
	findPointId(saved_graph_,car_in_piexl,start_id);

	if(pathPlanner(start_id,end_id,saved_graph_,path_astar)) {
		publishPlan(saved_graph_,path_astar);
	}
}

void MapManager::readNodeTXT(std::vector<MapGraph>& Index) {
	MapGraph graph;
  std::ifstream node_file;

  std::string file_name_node;
	file_name_node = map_folder_ + std::to_string(map_number_) + ".txt";

  node_file.open(file_name_node);
  if(!node_file.is_open()) {
  	cout<<"Could not find Node File"<<endl;
  	return;
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
}

bool MapManager::pathPlanner(int Start_Id,int End_id,
	std::vector<MapGraph> Graph,std::vector<int>& Path) {

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

	int current_id;
	visited[Start_Id] = 0;
	costs[Start_Id] = 0;
	current_id = Start_Id;

	findNeighbor(current_id,Graph,neighbor);

	while(!isGoalReached_) { //!isGoalReached && !isPlanFailed
		// if(current_id == End_id) {
		// 	isGoalReached = true;
		// 	//break;
		// }
		Path.clear();
		for (int i = 0; i < neighbor.size() ; i++) {
			int next_id = neighbor[i];
			double new_cost = costs[current_id] + computeHeuristic(current_id-1,next_id-1,Graph);
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
	while(!isPathReady_) { //!path_astar_que.empty()
		if (next_point == 0) {
			isPathReady_ = true;
			break;
		}
		Path.push_back(next_point);
		next_point = visited[next_point];
	}
	return true;
}

void MapManager::findNeighbor(int Id,std::vector<MapGraph> Index,std::vector<int>& Neighbor) {
	Neighbor.clear();

	for (int i = 0; i < Index[Id-1].neighbor.size(); i++)
	{
		if( Index[Id-1].neighbor[i] == 0 ) continue;
		Neighbor.push_back(Index[Id-1].neighbor[i]);
	}
}

bool MapManager::readYamlFile(){
  std::ifstream yaml_file;
  yaml_file.open(map_folder_ + std::to_string(map_number_) + "_info.txt");
  if(!yaml_file.is_open()) {
  	cout<<"Could not find Yaml File"<<endl;
  	return false;
  }

  string str;
  int info_linenum = 0;
  if(isReadYaml_) return true;
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
	}

	if(info_linenum == 4) {
		//cout<<"Yaml Read Complete"<<endl;
		isReadYaml_ = true;
		return true;
	} else {
		//cout<<"Yaml Read Missing Element "<< 4 - info_linenum<<endl;
		isReadYaml_ = false;
		return false;
	}
	yaml_file.close(); 
}

void MapManager::goalToPixel(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output) {
	Output.x = (map_info_.height - ((Input.y - map_info_.origin.y)/map_info_.resolution)) * 0.2;
	Output.y = ((Input.x - map_info_.origin.x)/map_info_.resolution) * 0.2;
}

void MapManager::findPointId(std::vector<MapGraph>& Map,geometry_msgs::Point32 Input,int& Output) {
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

void MapManager::publishPlan(std::vector<MapGraph> Map,std::vector<int> Path) {
	sensor_msgs::PointCloud pointcloud;
	pointcloud.header.frame_id = "/map";

	nav_msgs::Path path;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/map";
	path.header.frame_id = "/map";

	for (int i = 0; i < Path.size(); i++) {
		geometry_msgs::Point32 point;

		point.x = (Map[Path[Path.size()-i-1]-1].position.y * 5) * map_info_.resolution 
			+ map_info_.origin.x;
		point.y = (map_info_.height - (Map[Path[Path.size()-i-1]-1].position.x * 5)) * map_info_.resolution 
			+ map_info_.origin.y;
		pointcloud.points.push_back(point);

		pose.header.seq = Path.size()-i-1;
		pose.pose.position.x = point.x;
		pose.pose.position.y = point.y;
		path.poses.push_back(pose);
	}

	plan_pub.publish(path);
	plan_point_pub.publish(pointcloud);
	isGoalSend_ = false;
}
