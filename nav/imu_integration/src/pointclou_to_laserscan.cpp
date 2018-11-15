#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>


const int N_SCANS = 16;

void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& lasercloudMsg)
{
    std::cout<<"get"<<std::endl;
    std::vector<int> scanStartInd(N_SCANS,0);
    std::vector<int> scanEndInd(N_SCANS,0);
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;

    pcl::fromROSMsg(*lasercloudMsg,laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn,laserCloudIn,indices);

    int cloudsize = laserCloudIn.points.size();

    std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(N_SCANS);//存储每条线的点云

    int count = cloudsize;
    pcl::_PointXYZI point_tem;

    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);//lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转
    std::cout << startOri <<std::endl;
    float endOri = -atan2(laserCloudIn.points[cloudsize - 1].y,//todo 个人理解是相当于求解与-X轴的夹角
                          laserCloudIn.points[cloudsize - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }
    bool halfPassed = false;//lidar扫描线是否旋转过半




    for(int i = 0; i < cloudsize; i++)
    {
        point_tem.x = laserCloudIn.points[i].y;
        point_tem.y = laserCloudIn.points[i].z;
        point_tem.z = laserCloudIn.points[i].x;

        float angle = atan(point_tem.y / sqrt(point_tem.x * point_tem.x + point_tem.z * point_tem.z)) * 180 / M_PI;
        int scanID;
        int rounded_angle = int(angle + (angle<0.0 ? -0.5: +0.5));

        if(rounded_angle > 0)
        {
            scanID = rounded_angle;
        }
        else{
            scanID = rounded_angle + N_SCANS -1;
        }

        if(scanID > (N_SCANS - 1) ||  scanID < 0)
        {
            count--;
            continue;
        }

        laserCloudScans[scanID].push_back(point_tem);
    }
    for(int i = 0;i < N_SCANS;i++)
    {
        std::cout << "scan " << i << " has " << laserCloudScans[i].points.size() << std::endl;
    }
    for(int i = 0;i < laserCloudScans[1].points.size();i++)
    {
        float ori = -atan2(laserCloudScans[1].points[i].x, laserCloudScans[1].points[i].z);
        std::cout << ori <<std::endl;
    }
    std::cout << "over" << std::endl;



}


int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    ros::Subscriber pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points",2,pointCloudHandler);
    ros::spin();

    return  0;

}
