//
// Created by gc on 18-9-7.
//
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <cmath>
#include "utility/utility.h"
using namespace Eigen;
using namespace std;

ros::Publisher pub_traformed_poindCloud;

sensor_msgs::PointCloud2 msg_out;



void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& lasercloudMsg)
{
    std::cout << lasercloudMsg->header.frame_id << std::endl;

    pcl::PointCloud<pcl::PointXYZ> laserClouIN;
    pcl::fromROSMsg(*lasercloudMsg,laserClouIN);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(laserClouIN,msg_out);

    msg_out.header.stamp = lasercloudMsg->header.stamp;
    msg_out.header.frame_id = "base_laser";

    pub_traformed_poindCloud.publish(msg_out);

}


int main(int argc,char ** argv)
{
    ros::init(argc,argv,"rspoints_test");
    ros::NodeHandle nh2;

    ros::Subscriber sub_lasercoud = nh2.subscribe<sensor_msgs::PointCloud2>("/rslidar_points",2,laserCloudHandler);
    pub_traformed_poindCloud = nh2.advertise<sensor_msgs::PointCloud2>("/my_rslidar_points",2);

    ros::spin();

    return 0;
}

