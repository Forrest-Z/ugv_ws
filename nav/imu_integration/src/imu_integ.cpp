#include <ros/ros.h> 
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <cmath>
#include "utility/utility.h"
using namespace Eigen;
using namespace std;

sensor_msgs::Imu  imu_out;

int init_imu_times = 0;

double roll_init = 0.0;
double pitch_init = 0.0;
double yaw_init = 0.0;

double roll_last;double  pitch_last;double yaw_last;
double roll_now; double  pitch_now; double yaw_now;

double angular_x_last = 0.0;
double angular_y_last = 0.0;
double angular_z_last = 0.0;

double angu_x_bias = 0.0;
double angu_y_bias = 0.0;
double angu_z_bias = 0.0;
double angu_x_bias_sum = 0.0;
double angu_y_bias_sum = 0.0;
double angu_z_bias_sum = 0.0;

double linear_acc_sum_x = 0.0;
double linear_acc_sum_y = 0.0;
double linear_acc_sum_z = 0.0;

double linear_acc_x = 0.0;
double linear_acc_y = 0.0;
double linear_acc_z = 0.0;

double imu_time_last;
bool initilized = false;
Eigen::Quaterniond q_start(1,0,0,0);
Eigen::Quaterniond q_end;

ros::Publisher Imu_pub;



Quaterniond euler2quaternion(Vector3d euler)
{
    double cr = std::cos(euler(0)/2);
    double sr = sin(euler(0)/2);
    double cp = cos(euler(1)/2);
    double sp = sin(euler(1)/2);
    double cy = cos(euler(2)/2);
    double sy = sin(euler(2)/2);
    Quaterniond q;
    q.w() = cr*cp*cy + sr*sp*sy;
    q.x() = sr*cp*cy - cr*sp*sy;
    q.y() = cr*sp*cy + sr*cp*sy;
    q.z() = cr*cp*sy - sr*sp*cy;
    return q;
}

int imu_integration()
{
    if(!initilized)//初始化
    {
        if( init_imu_times < 1000)//需要静置4秒钟，计算陀螺仪零偏
        {
            init_imu_times++;
            angu_x_bias_sum += imu_out.angular_velocity.x;
            angu_y_bias_sum += imu_out.angular_velocity.y;
            angu_z_bias_sum += imu_out.angular_velocity.z;

            linear_acc_sum_x += imu_out.linear_acceleration.x;
            linear_acc_sum_y += imu_out.linear_acceleration.y;
            linear_acc_sum_z += imu_out.linear_acceleration.z;
            return 0;
        }
        angu_x_bias = angu_x_bias_sum/1000.0;
        angu_y_bias = angu_y_bias_sum/1000.0;
        angu_z_bias = angu_z_bias_sum/1000.0;
        std::cout<< angu_x_bias << std::endl;

        //求解初始roll,pitch
        //1.求得各轴的加速度测量值
        double init_acc_x = linear_acc_sum_x/1000.0;
        double init_acc_y = linear_acc_sum_y/1000.0;
        double init_acc_z = linear_acc_sum_z/1000.0;
        //计算初始yaw,pitch
        roll_init = std::asin(init_acc_x/9.80);
        pitch_init = std::atan2(-init_acc_y,init_acc_z);
        Eigen::Vector3d rpy(roll_init,pitch_init,yaw_init);
        //初始姿态的四元数表示
        q_start = euler2quaternion(rpy);


        angular_x_last = imu_out.angular_velocity.x;
        angular_y_last = imu_out.angular_velocity.y;
        angular_z_last = imu_out.angular_velocity.z;

        imu_time_last = imu_out.header.stamp.toSec();
        initilized = true;

    }
    else{
        /*********************陀螺仪积分求解姿态****************************/
        // 陀螺仪计算角度变化量
        double angular_x_now = imu_out.angular_velocity.x;
        double angular_y_now = imu_out.angular_velocity.y;
        double angular_z_now = imu_out.angular_velocity.z;

        double imu_time_now = imu_out.header.stamp.toSec();
        double d_t = imu_time_now - imu_time_last;
        //std::cout << d_t << std::endl;

        double d_angule_x = d_t * ( angular_x_last + angular_x_now - 2 * angu_x_bias)/2.0;//后面此处添加bias
        double d_angule_y = d_t * ( angular_y_last + angular_y_now - 2 * angu_y_bias)/2.0;
        double d_angule_z = d_t * ( angular_z_last + angular_z_now - 2 * angu_z_bias)/2.0;
        //此处角度叠加
        q_end = q_start * Eigen::Quaterniond(1,d_angule_x,d_angule_y,d_angule_z);//陀螺仪在上一次结果上叠加积分结果
        q_end.normalize();
        double angu_roll, angu_pitch, angu_yaw;//角速度积分得到的角度值
        tf::Matrix3x3(tf::Quaternion(q_end.x(),q_end.y(),q_end.z(),q_end.w())).getRPY(angu_roll, angu_pitch, angu_yaw);//todo 陀螺仪计算得到的角度
        //std::cout<< q_end.x() << q_end.y() << q_end.z() << q_end.w() << std::endl;
        /**********************陀螺仪积分结束***************************************/

        imu_out.orientation.x = q_end.x();
        imu_out.orientation.y = q_end.y();
        imu_out.orientation.z = q_end.z();
        imu_out.orientation.w = q_end.w();
        imu_out.header.frame_id = "base_link";

        /***************加速度求解imu姿态**********************/
        linear_acc_x = imu_out.linear_acceleration.x;
        linear_acc_y = imu_out.linear_acceleration.y;
        linear_acc_z = imu_out.linear_acceleration.z;
        double roll_acc = std::asin(linear_acc_x/9.80);//todo 加速度计算得到的角度
        double pitch_acc = std::atan2(-linear_acc_y,linear_acc_z);
        /*************************加速度计算角度结束****************************/

        /*******************两种方法计算出来的角度融合  todo:互补滤波 ********************/
        double k = 0.90;//融合系数
        double fused_roll = k * angu_roll  +  (1 - k) * roll_acc;
        double fused_pitch = k *angu_pitch +  (1 - k) * pitch_acc;
        double fused_yaw = angu_yaw;//yaw角没有融合

        Eigen::Vector3d rpy(fused_roll,fused_pitch,fused_yaw);
        Quaterniond q_fused = euler2quaternion(rpy);

        imu_out.orientation.x = q_fused.x();
        imu_out.orientation.y = q_fused.y();
        imu_out.orientation.z = q_fused.z();
        imu_out.orientation.w = q_fused.w();

        Imu_pub.publish(imu_out);

        //数据转换存储
        imu_time_last = imu_time_now;
        q_start = q_fused;
        angular_x_last = angular_x_now;
        angular_y_last = angular_y_now;
        angular_z_last = angular_z_now;
    }


}



void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{

    //std::cout<< "angular_velocity.x =  "<< imuIn->angular_velocity.x << std::endl;

    imu_out.angular_velocity.x = imuIn->angular_velocity.x;
    imu_out.angular_velocity.y = imuIn->angular_velocity.y;
    imu_out.angular_velocity.z = imuIn->angular_velocity.z;

    ofstream ofile;
    ofile.open("/home/gc/Desktop/acc.txt",ios::app);
    ofile << imuIn->linear_acceleration.x <<"  " << imuIn->linear_acceleration.y <<  "  " << imuIn->linear_acceleration.z <<endl;
    ofile.close();
    imu_out.linear_acceleration.x = imuIn->linear_acceleration.x;
    imu_out.linear_acceleration.y = imuIn->linear_acceleration.y;
    imu_out.linear_acceleration.z = imuIn->linear_acceleration.z;
    imu_out.header.stamp = imuIn->header.stamp;

    imu_out.header.stamp = imuIn->header.stamp;

    imu_integration();


}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"imu_integ");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/zr300_node/imu",50,imuHandler);
    Imu_pub = nh.advertise<sensor_msgs::Imu>("/IMU_data", 20);;
    ros::spin();
    return 0;
}

 
/*int main(int argc, char** argv)
{
    // Step 2: Initialization:
       ros::init(argc, argv, "imu");     
    ros::NodeHandle n;  
 
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);  
    ros::Rate loop_rate(50);  
    while(ros::ok())
    {
                   
            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";
            //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
           imu_data.orientation.x = 0;
            imu_data.orientation.y = -1;
            imu_data.orientation.z = -5;
            imu_data.orientation.w = 6;
            //线加速度
            imu_data.linear_acceleration.x = 0.01; 
            imu_data.linear_acceleration.y = 0.02;
            imu_data.linear_acceleration.z = 0.03;
	    //角速度
            imu_data.angular_velocity.x = 0.05; 
            imu_data.angular_velocity.y = 0.06; 
            imu_data.angular_velocity.z = 0.07;
 
 
 
            IMU_pub.publish(imu_data);
       
 
       
        ros::spinOnce();  
        loop_rate.sleep();  
    }
 
    return 0;
}*/
