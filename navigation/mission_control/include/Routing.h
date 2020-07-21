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


using std::vector;
using std::queue;
using std::cout;
using std::endl;
using std::string;

struct PositionPiexl {
  int x;
  int y;
  int z;
};

struct Position {
  double x;
  double y;
  double z;
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

  void RoutingAnalyze(geometry_msgs::Point32& goal_in_map,geometry_msgs::Point32 vehicle_in_map,string map_folder,int map_number);
  sensor_msgs::PointCloud path_pointcloud(){ return path_pointcloud_; };

  void ModifyRoutePoint(geometry_msgs::Point32 Input,sensor_msgs::PointCloud& Output,sensor_msgs::PointCloud Obstacle);
private:
  /** Parameters **/
  std::string map_folder_;

  /** Flags **/
  bool isPathReady_;
  bool isGoalReached_;

  /** Variables **/
  YamlInfo map_info_;
  vector<MapGraph> node_info_;
  vector<int> path_;
  sensor_msgs::PointCloud path_pointcloud_;

  vector<RoutingBlock> routing_info_; 

  /** Functions **/
  bool ReadNodeTXT(string Map_folder,int Map_number);
  bool ReadYamlFile(string Map_folder,int Map_number);

  int FindPointId(geometry_msgs::Point32 Input);

  bool ComputePath(int Start_Id,int End_id);
  void SetPathtoPointcloud();
  void CleanAllState();

  /** Inline Function **/ 
  inline bool checkIndexVaild(int input,std::vector<MapGraph> Index) {
    return ( input > 0 && input <= Index.size() );
  }

  inline double computeHeuristic(int Start_Id,int End_id,std::vector<MapGraph> Index) {
    double distances_cost = hypot(abs(Index[End_id-1].position.x-Index[Start_Id-1].position.x)
        ,abs(Index[End_id-1].position.y-Index[Start_Id-1].position.y));
     double global_cost = computeRoutingCost(Start_Id,End_id);
     return (distances_cost + global_cost);
  }

  inline double computeRoutingCost(int id_1,int id_2) {
    for (int i = 0; i < routing_info_.size(); ++i) {
      if( id_1 == routing_info_[i].id_1  || id_2 == routing_info_[i].id_1 ) {
          return routing_info_[i].cost;
      }
    }
    return 0;
  }

  inline void findNeighbor(int Id,std::vector<MapGraph> Index,std::vector<int>& Neighbor) {
    Neighbor.clear();

    for (int i = 0; i < Index[Id-1].neighbor.size(); i++)
    {
      if( Index[Id-1].neighbor[i] == 0 ) continue;
      Neighbor.push_back(Index[Id-1].neighbor[i]);
    }
  }

  inline geometry_msgs::Point32 ConvertMapToPixel(geometry_msgs::Point32 Input) {
    geometry_msgs::Point32 output;
    output.x = (map_info_.height - ((Input.y - map_info_.origin.y)/map_info_.resolution)) * map_info_.ratio;
    output.y = ((Input.x - map_info_.origin.x)/map_info_.resolution) * map_info_.ratio;
    return output;
  }

  
};


#endif
