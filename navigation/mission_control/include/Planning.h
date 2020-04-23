#ifndef LIB_PLANNING_H
#define LIB_PLANNING_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <time.h>
#include <limits.h>
#include <algorithm> 
#include <mutex>


#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_datatypes.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;

struct CandidatePixel {
  int index;
  int width;
  int grade;
};

class Planning
{
public:
	Planning(){
		path_window_radius_    = 8;
		path_swap_range_       = 1.2; // 1.2
		path_vertical_step_    = 0.2;

		splines_joints_num_    = 3;

		costmap_resolution_    = 0.4;
	 	safe_path_search_grid_ = 5;

	 	path_window_radius_standard_ = path_window_radius_;
	}
	~Planning(){}

	bool GenerateCandidatePlan(geometry_msgs::Point32 Goal,sensor_msgs::PointCloud Obstacle);
	bool SelectBestPath(vector<vector<sensor_msgs::PointCloud>> Path_set_2d,geometry_msgs::Point32 Goal);
	bool CheckIdealPath(geometry_msgs::Point32& Goal);
	bool UpdateCostmap(sensor_msgs::PointCloud Obstacle);
	vector<sensor_msgs::PointCloud> path_all_set() {return path_all_set_;}; 
	vector<sensor_msgs::PointCloud> path_safe_set() {return path_safe_set_;};
	vector<sensor_msgs::PointCloud> path_debug_set() {return path_debug_set_;};
	sensor_msgs::PointCloud path_best() {return path_best_;};
	nav_msgs::OccupancyGrid costmap_local() {return costmap_local_;};
	double path_window_radius_standard() {return path_window_radius_standard_;};
	void set_path_window_radius_standard(double Input) {path_window_radius_standard_ = Input;};
	void set_costmap_resolution(double Input) {costmap_resolution_ = Input;};
	void set_safe_path_search_grid(double Input) {
		mtx_radius_.lock();
		safe_path_search_grid_ = Input;
		mtx_radius_.unlock();
	};
	// int GetPathSize(return )

private:
	const double PI = 3.14159265359;

	/** Parameters **/
	double path_window_radius_;
	double path_window_radius_standard_;
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
	vector<sensor_msgs::PointCloud> path_debug_set_;
	sensor_msgs::PointCloud path_best_;
	sensor_msgs::PointCloud path_standard_;
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
	bool DetectObstcaleGrid(sensor_msgs::PointCloud Path,nav_msgs::OccupancyGrid Costmap);
	double ComputePathCost(sensor_msgs::PointCloud Path,geometry_msgs::Point32 Goal);
	sensor_msgs::PointCloud ComputeArcPoints(geometry_msgs::Point32 Origin,double Radius,double Start_Rad,double End_Rad,int Nums);

	geometry_msgs::Point32 FindSafePoint(geometry_msgs::Point32 Input,nav_msgs::OccupancyGrid Costmap);

	/** Inline Function **/ 
	inline int ConvertCartesianToLocalOccupany(nav_msgs::OccupancyGrid Grid,geometry_msgs::Point32 Point) {
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

	

};
#endif