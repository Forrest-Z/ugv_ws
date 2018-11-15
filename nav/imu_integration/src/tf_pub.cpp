//
// Created by gc on 18-9-11.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//#include <turtlesim/Pose.h>

std::string turtle_name;



void odom_Callback(const nav_msgs::OdometryConstPtr msg){


    //设置odom与base_link之间的tf
    static tf::TransformBroadcaster br_odom_baselink;
    tf::Transform transform;

    std::cout << "the frame of wheel_odom is : " << msg->header.frame_id <<std::endl;
    //设置平移量
    std::cout<< "x向位移： " << msg->pose.pose.position.x << " y向位移" << msg->pose.pose.position.y << std::endl;
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );

    //设置旋转量
    double yaw = acos(msg->pose.pose.orientation.w);
    tf::Quaternion q;
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
    transform.setRotation(q);

    br_odom_baselink.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));

    //设置base_link与base_laser之间的tf
    static tf::TransformBroadcaster br_baselink_baselaser;
    tf::Transform transform_1;

    transform_1.setOrigin(tf::Vector3(0.1,0,0.7));
    tf::Quaternion q_1;
    q_1.setX(0);
    q_1.setY(0);
    q_1.setZ(0);
    q_1.setW(1);
    transform_1.setRotation(q_1);

    br_baselink_baselaser.sendTransform(tf::StampedTransform(transform_1,msg->header.stamp,"base_link","base_laser"));


}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
   // if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    //turtle_name = argv[1];

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/wheel_odom", 10, &odom_Callback);

    ros::spin();
    return 0;
};

