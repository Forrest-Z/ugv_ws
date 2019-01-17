#ifndef LIB_MAPMANAGER_H
#define LIB_MAPMANAGER_H

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

const int ROS_RATE_HZ = 20;


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
};

struct NodeCompare {
  bool operator()(const MapGraph &na, const MapGraph &nb) {
    if (na.cost == nb.cost) return (na.id <= nb.id);
    return(na.cost<= nb.cost);
  }

};


class MapManager
{
public:
  MapManager();
  ~MapManager();

  void Manager();
  void Mission();
  void Initialization();

private:
  /** Node Handles **/
  ros::NodeHandle n;
  ros::NodeHandle pn;

  /** Publishers **/
  ros::Publisher plan_pub;
  ros::Publisher plan_point_pub;
  /** Subscribers **/
  ros::Subscriber goal_sub;
  ros::Subscriber map_number_sub;

  /** Parameters **/
  std::string map_folder_;
  tf::TransformListener listener;

  /** Flags **/
  bool isPathReady_;
  bool isGoalReached_;
  bool isGoalSend_;
  bool isReadYaml_;
  /** Variables **/
  // std::mutex mutex_draw_;
  vector<MapGraph> saved_graph_;
  YamlInfo map_info_;
  geometry_msgs::Point32 goal_in_map_;

  int map_number_;

  /** Functions **/
  void debugFunction();

  void readNodeTXT(std::vector<MapGraph>& Index);
  void routingAnalyze();
  bool readYamlFile();
  void goalToPixel(geometry_msgs::Point32 Input,geometry_msgs::Point32& Output);
  void findPointId(std::vector<MapGraph>& Map,geometry_msgs::Point32 Input,int& Output);
  bool pathPlanner(int Start_Id,int End_id,
    std::vector<MapGraph> Index,std::vector<int>& Path);
  void publishPlan(std::vector<MapGraph> Map,std::vector<int> Path);

  void findNeighbor(int Id,std::vector<MapGraph> Index,std::vector<int>& Neighbor);
  inline bool checkIndexVaild(int input,std::vector<MapGraph> Index) {
    return ( input > 0 && input <= Index.size() );
  }
  inline double computeHeuristic(int Start_Id,int End_id,std::vector<MapGraph> Index) {
    return ( hypot(abs(Index[End_id].position.x-Index[Start_Id].position.x),abs(Index[End_id].position.y-Index[Start_Id].position.y)) );
  }

  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& input) {
    goal_in_map_.x = input->pose.position.x;
    goal_in_map_.y = input->pose.position.y;
    isGoalSend_ = true;
    isReadYaml_ = false;
    isGoalReached_ =false;
    isPathReady_ = false;
    // cout<<"New Goal"<<endl;
  }

  void map_number_callback(const std_msgs::Int32::ConstPtr& input) {
    map_number_ = input->data;;
  }
  
};


#endif
