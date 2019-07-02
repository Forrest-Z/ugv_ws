#ifndef LIB_ROUTING_H
#define LIB_ROUTING_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <time.h>
#include <mutex>
#include <queue>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


using std::vector;
using std::queue;
using std::cout;
using std::endl;
using std::string;

struct PositionPiexl {
  int x;
  int y;
};

struct Position {
  double x;
  double y;
};

struct MapGraph {
  int id;
  PositionPiexl position;
  vector<int> neighbor;
  double cost;
};

struct YamlInfo {
  double resolution;
  Position origin;
  double width;
  double height;
  double ratio;
};

struct RoutingBlock {
  int id_1;
  int id_2;
  double cost;
};

struct NodeCompare {
  bool operator()(const MapGraph &na, const MapGraph &nb) {
    if (na.cost == nb.cost) return (na.id <= nb.id);
    return(na.cost<= nb.cost);
  }

};


class Routing
{
public:
  Routing(){};
  ~Routing(){};

  void routingAnalyze(geometry_msgs::Point32 goal_in_map,string map_folder,int map_number);
  void getPath(std::vector<MapGraph>& Map,std::vector<int>& Path,YamlInfo& Map_info);

private:

  /** Parameters **/
  std::string map_folder_;
  tf::TransformListener listener;

  /** Flags **/
  bool isPathReady_;
  bool isGoalReached_;
  /** Variables **/
  YamlInfo map_info_;

  std::vector<MapGraph> map_;
  std::vector<int> path_;
  std::vector<RoutingBlock> routing_info_; 


  /** Functions **/
  bool readNodeTXT(std::vector<MapGraph>& Index,string map_folder,int map_number);
  bool readYamlFile(string map_folder,int map_number);

  void mapToPixel(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output);
  void findPointId(std::vector<MapGraph>& Map,geometry_msgs::Point32 Input,int& Output);
  bool pathPlanner(int Start_Id,int End_id,
    std::vector<MapGraph> Index,std::vector<int>& Path);

  void findNeighbor(int Id,std::vector<MapGraph> Index,std::vector<int>& Neighbor);
  double computeHeuristic(int Start_Id,int End_id,std::vector<MapGraph> Index);
  double computeRoutingCost(int id_1,int id_2);

  inline bool checkIndexVaild(int input,std::vector<MapGraph> Index) {
    return ( input > 0 && input <= Index.size() );
  }

  
};


#endif
