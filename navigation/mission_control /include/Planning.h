#ifndef LIB_PLANNING_H
#define LIB_PLANNING_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <list>
#include <time.h>
#include <limits.h>
#include <algorithm> 
#include <mutex>


#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::list;
using std::random_device;


struct CandidatePixel {
  int index;
  int width;
  int grade;
};

struct AstarPoint
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	double F, G, H; //F=G+H
	AstarPoint *parent; //parent的坐标
	AstarPoint(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {}  //变量初始化
};

struct RRTPointCell
{
  double x, y;
};
    
struct RRTTreeNode
{
  RRTPointCell cell;
  RRTTreeNode* father;
  vector<RRTTreeNode*> children;
};

class Planning
{
public:
	Planning(){
		map_window_radius_     				= 8;
		path_swap_range_       				= 1.2;
		path_vertical_step_    				= 0.2;

		splines_joints_num_    				= 3;

		costmap_resolution_    				= 0.2; // xiugai
	 	safe_path_search_grid_ 				= 5;

		Astar_local_map_window_radius_ 		= 30;
    	Astar_local_costmap_resolution_ 	= 0.5;
		Astar_plan_num_ 					= 1;

		RRT_iteration_ 						= 5000;
		RRT_sample_num_ 					= 5;
		RRT_extend_step_ 					= 0.2;
		RRT_threhold_ 						= 0.5;
		RRT_probability_ 					= 0.8;
		RRT_search_plan_num_ 				= 1;
		RRT_expand_size_ 					= 2;

		path_best_.header.frame_id = "/base_link";
	}
	~Planning(){}
	bool GenerateCandidatePlan(geometry_msgs::Point32 Goal,sensor_msgs::PointCloud Obstacle,double Path_Radius);
	bool SelectBestPath(vector<vector<sensor_msgs::PointCloud>> Path_set_2d,geometry_msgs::Point32 Goal);
	bool CheckIdealPath(geometry_msgs::Point32& Goal);
	bool UpdateCostmap(sensor_msgs::PointCloud Obstacle);

	vector<sensor_msgs::PointCloud> path_all_set() {return path_all_set_;}; 
	vector<sensor_msgs::PointCloud> path_safe_set() {return path_safe_set_;};
	sensor_msgs::PointCloud path_best() {return path_best_;};
	nav_msgs::OccupancyGrid costmap_local() {return costmap_local_;};
	double map_window_radius() {return map_window_radius_;};

	void set_map_window_radius(double Input) {map_window_radius_ = Input;};
	void set_path_swap_range(double Input) {path_swap_range_ = Input;};
	void path_vertical_step(double Input) {path_vertical_step_ = Input;};
	void set_costmap_resolution(double Input) {costmap_resolution_ = Input;};
	void set_safe_path_search_grid(double Input) {
		mtx_radius_.lock();
		safe_path_search_grid_ = Input;
		mtx_radius_.unlock();
	};

	void set_Astar_map_window_radius(double Input) {Astar_local_map_window_radius_ = Input;}
	void set_Astar_costmap_resolution(double Input) {Astar_local_costmap_resolution_ = Input;}
	void set_RRT_iteration(int Input) {RRT_iteration_ = Input;}
	void set_RRT_exten_step(double Input) {RRT_extend_step_ = Input;}
	void set_RRT_search_plan_num(double Input) {RRT_search_plan_num_ = Input;}
	void set_RRT_expand_size(double Input) {RRT_expand_size_ = Input;}


	inline int  ConvertCartesianToLocalOccupany(nav_msgs::OccupancyGrid Grid,geometry_msgs::Point32 Point) {
    geometry_msgs::Point32 temp_point;
    temp_point.x = Point.x - Grid.info.origin.position.x;
    temp_point.y = Point.y - Grid.info.origin.position.y;

    int row_num = temp_point.x / Grid.info.resolution;
    int col_num = temp_point.y / Grid.info.resolution;

    int costmap_size = Grid.info.width * Grid.info.height;
    int output = col_num * Grid.info.width + row_num;

    if(output > (costmap_size-1)) output = -1;
    return output;
  }

  inline geometry_msgs::Point32 ConvertLocalOccupanyToCartesian(nav_msgs::OccupancyGrid Grid,int Index) {
  	geometry_msgs::Point32 output;

  	int col_num = Index / Grid.info.width;
  	int row_num = Index % Grid.info.width;

  	output.x = row_num * Grid.info.resolution + Grid.info.origin.position.x;
  	output.y = col_num * Grid.info.resolution + Grid.info.origin.position.y;

  	return output;
  }

  //====================Astar Pathfind==============================
    void SetAstarLocalCostmap(nav_msgs::OccupancyGrid& Costmap,sensor_msgs::PointCloud Obstacle);
    void InitAstarLocalCostmap(nav_msgs::OccupancyGrid& Costmap);
    geometry_msgs::Point32 ConvertCartesianToAstarArray(geometry_msgs::Point32 Point);
    nav_msgs::OccupancyGrid GetAstarLocalCostmap() {return Astar_local_costmap_;}
    bool UpdateAstarCostmap(sensor_msgs::PointCloud Obstacle);
    vector<sensor_msgs::PointCloud> getAstarPath(geometry_msgs::Point32 start_point,geometry_msgs::Point32 end_point);
	bool isAstarVaildPoint(const AstarPoint *point);
	void SetAstarCostmapPath(vector<list<AstarPoint> > path_costmap, signed char path_occupancy);
	bool AstarPathCost(list<AstarPoint> cost_path);
	geometry_msgs::Point32 ConvertAstarArrayToCartesian(geometry_msgs::Point32 Point);
	
  //====================RRT Pathfind==============================
	nav_msgs::Path getRRTPlan(const geometry_msgs::Point32 start, const geometry_msgs::Point32 goal);
	void RRTExpandCostmap(nav_msgs::OccupancyGrid &Grid);
	void RRTInitialRoot(const geometry_msgs::Point32 start);
	bool BuildRRT(geometry_msgs::Point32 goal);
	bool RRTExtendNearestNode(RRTPointCell random,geometry_msgs::Point32 goal);
	bool RRTExtendSingleStep(const RRTTreeNode* rrtnode, RRTTreeNode* &node, const RRTPointCell random, geometry_msgs::Point32 goal);
	void RRTProbSample(geometry_msgs::Point32 goal);
	void RRTAddGoalOrientNode(geometry_msgs::Point32 goal);
	void RRTAddRandomNode();
	bool RRTGoalReached(const RRTTreeNode* nearestNode, geometry_msgs::Point32 goal);
	void SetRRTPath();
	void SetRRTTree();
	nav_msgs::Path getRRTPath() {return RRT_path_;}
	nav_msgs::Path getRRTCurvePath() {return RRT_Curve_path_;}
	vector<nav_msgs::Path> getRRTPathQue() {return RRT_path_que_;}
	sensor_msgs::PointCloud getRRTTree() {return RRT_tree_;}
	nav_msgs::Path ComputeBSplineCurve(nav_msgs::Path path);


private:
	const double PI = 3.14159265359;
	const int ADDITION_IDEAL = 2;
	const int ADDITION_SPLINE = 0;

	/** Parameters **/
	double path_window_radius_;
	double map_window_radius_;
	double path_swap_range_;
	double path_vertical_step_;

	int splines_joints_num_;

	double costmap_resolution_;
	int safe_path_search_grid_;

	std::mutex mtx_radius_;

	/** Flag **/

	/** Variables **/
	vector<sensor_msgs::PointCloud> path_all_set_;
	vector<sensor_msgs::PointCloud> path_safe_set_;
	sensor_msgs::PointCloud path_best_;
	nav_msgs::OccupancyGrid costmap_local_;

	/** Functions **/

	void GenerateSplinesJoints(vector<sensor_msgs::PointCloud>& Output);

	void GenerateSplinesPath(vector<sensor_msgs::PointCloud>& Path_set,vector<sensor_msgs::PointCloud> Joints);
	void GenerateSplinesPath(vector<sensor_msgs::PointCloud>& Path_set,vector<sensor_msgs::PointCloud> Joints, vector<vector<sensor_msgs::PointCloud>>& Path_set_2d);
	void ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_0,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2,geometry_msgs::Point32 Point_3,double Level);
	void ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2, double Level);
	void ComputeSplines(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Point_1,geometry_msgs::Point32 Point_2,geometry_msgs::Point32 Point_3,double Level);
	void ComputeStraight(sensor_msgs::PointCloud& Output,geometry_msgs::Point32 Goal);
	void InitLocalCostmap(nav_msgs::OccupancyGrid& Costmap);
	void SetLocalCostmap(nav_msgs::OccupancyGrid& Costmap,sensor_msgs::PointCloud Obstacle);
	void ComputeSafePath(vector<vector<sensor_msgs::PointCloud>> Input_2d,vector<vector<sensor_msgs::PointCloud>>& Output_2d,vector<sensor_msgs::PointCloud>& Output,nav_msgs::OccupancyGrid Costmap);
	bool DetectObstcaleGrid(sensor_msgs::PointCloud Path,nav_msgs::OccupancyGrid Costmap,int Addition);

	/** Inline Function **/ 

  inline bool CheckGrid(nav_msgs::OccupancyGrid Grid,int Index,signed char Threshold) {
    int grid_size = Grid.info.width * Grid.info.height;
    if(Index < grid_size && Index > -1) {
      if(Grid.data[Index] > Threshold) return false;
    }
    return true;
  }

  inline int ComputeDigit(double Input,int Digit) {
  	if(Input < pow(10,Digit-1)) return 0;
  	int reminder = int(Input) % int(pow(10,Digit));
  	int output = reminder/pow(10,Digit-1);
  	return output;
  }

//=================Astar Pathfind=================================
	const double Astar_cost_1 = 0.4; //直移一格消耗
	const double Astar_cost_2 = 0.56; //斜移一格消耗
	const int INFINITE = 1000;

	int Astar_plan_num_;
    double Astar_local_map_window_radius_;
    double Astar_local_costmap_resolution_;

    nav_msgs::OccupancyGrid Astar_local_costmap_;
    vector<vector<signed char> > Astar_costmap_array_;  //搜索栅格
	vector<sensor_msgs::PointCloud> Astar_path_;
	vector<sensor_msgs::PointCloud> Astar_all_path_;
	vector<list<AstarPoint> > Astar_all_array_;
	vector<list<AstarPoint> > Astar_array_;


	list<AstarPoint *> Astar_open_list_;  //开启列表
	list<AstarPoint *> Astar_close_list_; //关闭列表


    AstarPoint *findAstarPath(AstarPoint &startPoint, AstarPoint &endPoint);
	vector<AstarPoint *> getAstarSurroundPoints(const AstarPoint *point);
	bool isAstarCanreach(const AstarPoint *point, const AstarPoint *target);
	AstarPoint *isInAstarList(const list<AstarPoint *> &list, const AstarPoint *point); //判断开启/关闭列表中是否包含某点
	AstarPoint *getAstarLeastFpoint(); //从开启列表中返回F值最小的节点
	//计算FGH值
	double calcAstarG(AstarPoint *temp_start, AstarPoint *point);
	double calcAstarH(AstarPoint *point, AstarPoint *start, AstarPoint *end);
	double calcAstarF(AstarPoint *point);
	

  //====================RRT Pathfind==============================
	RRTTreeNode* RRT_root_;//这是整棵树的根部，即起点位置
	RRTTreeNode* RRT_goal_node_;//终点位置

	nav_msgs::OccupancyGrid RRT_costmap_local_;

	/* rrt elements */
	double RRT_threhold_, RRT_probability_;//判断到终点的容忍度，以及向终点方向拉扯的撒点概率
	random_device RRT_rd_;//随机数生成器
	int RRT_iteration_;//最大迭代次数
	int RRT_sample_num_;//每次随机撒点的数量
	double RRT_extend_step_;//树枝扩展的步长
	vector<RRTTreeNode*> RRT_nodes_;
	vector<RRTPointCell> RRT_current_sampling_;//本次撒点的样本

	/*rrt path and pointcloud*/
	sensor_msgs::PointCloud RRT_tree_;
	nav_msgs::Path RRT_path_;
	nav_msgs::Path RRT_Curve_path_;
	vector<nav_msgs::Path> RRT_path_que_;

	int RRT_search_plan_num_;
	int RRT_expand_size_;

	inline void RRTClear() {
		for(int i = 0; i < RRT_nodes_.size(); i++) {
        	delete RRT_nodes_[i];
        	RRT_nodes_[i] = NULL;
    	}
  	}

  	inline int ComputeFactorial(int n) {
    	if(n == 0) return 1;
    	int n_factorial = 1;
    	for(int i = 1; i <= n; i++) {
      		n_factorial = n_factorial * i;
    	}
    	return n_factorial;
  	}

};
#endif