//#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){
    ros::init(argc,argv, "speed_control");

    

    ros::NodeHandle n;
    geometry_msgs::Twist cmdvel;
    ros::Publisher pub_speed;

    pub_speed = n.advertise<geometry_msgs::Twist>("/cmd_vel",5);
    cmdvel.linear.x=0.2;
    //cmdvel.angular.z=0.1;

    ros::Rate loop_rate(10);
    while(ros::ok()){
        pub_speed.publish(cmdvel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
