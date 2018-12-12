#ifndef LIB_OBSTACLEMANAGER_H
#define LIB_OBSTACLEMANAGER_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::vector;
using std::cout;
using std::endl;

const int ROS_RATE_HZ   = 1;

const vector<double> R11 = {4.619579111799e-01, 1.120235603441e-01, 8.797986191318e-01};
const vector<double> R12 = {-8.847258557293e-01, -1.123975717332e-02, 4.659762097605e-01};
const vector<double> R13 = {6.208903689347e-02, -9.936419827013e-01, 9.391784554130e-02};
const vector<double> T1  = {2.566721252215e-02, 1.607875013048e-01, -3.012084046145e-01 };

const vector<double> R21 = {0.911020853031573, 0.00519415403124045, 0.412327571362292};
const vector<double> R22 = {-0.00168139473597834, 0.999959143617621, -0.00888166691871711};
const vector<double> R23 = {-0.412356857895401, 0.00739809836464467, 0.910992365438492};
const vector<double> T2  = {-10.6632863659008, 12.5020142707178, 2.33286297823336};

const vector<double> R31 = {0.911020853031573,-0.00168139473597833,-0.412356857895401};
const vector<double> R32 = {0.00519415403124034,0.999959143617621,0.00739809836464456};
const vector<double> R33 = {0.412327571362291,-0.00888166691871699,0.910992365438493};
const vector<double> T3  = {10.6632863659008,-12.5020142707178,-2.33286297823336};


class ObstacleManager
{
public:
	ObstacleManager();
	~ObstacleManager();

	void Manager();
	void Mission();

private:
	/** Node Handles **/
	ros::NodeHandle n;
	ros::NodeHandle pn;

	tf2_ros::TransformBroadcaster br1_;
	tf2_ros::TransformBroadcaster br2_;
	tf2_ros::TransformBroadcaster br3_;

	void tfPublish();
	
};


#endif

		/*
		#calib_time: Mon Dec 10 04:13:16 2018
		#description: use R/T/S to transform campoint c into laser coordinates l=R*S*c+T

		R: 
		4.619579111799e-01 1.120235603441e-01 8.797986191318e-01 
		-8.847258557293e-01 -1.123975717332e-02 4.659762097605e-01 
		6.208903689347e-02 -9.936419827013e-01 9.391784554130e-02

		T: 
		2.566721252215e-02 1.607875013048e-01 -3.012084046145e-01

		S: 
		1.000000000000e+00



		roll  tan^-1 (32/33)
		pitch tan^-1 ( -31/sqrt(32^2+33^))
		yaw   tan^-1 (21/11)

		

		double LeMat1[9] = { 
			0.911020853031573, 0.00519415403124045, 0.412327571362292,
			-0.00168139473597834, 0.999959143617621, -0.00888166691871711,
			-0.412356857895401, 0.00739809836464467, 0.910992365438492 };  //B 
		double LeVector1[4] = { -10.6632863659008, 12.5020142707178, 2.33286297823336 ,1000 };


	"RightRotationMatrix": [0.911020853031573,-0.00168139473597833,-0.412356857895401,
	0.00519415403124034,0.999959143617621,0.00739809836464456,
	0.412327571362291,-0.00888166691871699,0.910992365438493],
	"RightTranslationVector": [10.6632863659008,-12.5020142707178,-2.33286297823336],



		*/