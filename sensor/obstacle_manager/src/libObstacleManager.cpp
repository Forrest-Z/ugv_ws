#include "libObstacleManager.h"

ObstacleManager::ObstacleManager():pn("~")
{
  sleep(1);
}


ObstacleManager::~ObstacleManager(){
}

void ObstacleManager::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  while (ros::ok()) {
    ObstacleManager::Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void ObstacleManager::Mission() {
	tfPublish();
}

void ObstacleManager::tfPublish() {
	int invert_index = 1;

	geometry_msgs::TransformStamped transformStamped1;
	tf2::Quaternion q1;	
	double roll1,pitch1,yaw1;
  invert_index = 1;

	roll1 = atan2(R13[1],R13[2]);
  pitch1 = atan2(-R13[0],hypot(R13[1],R13[2]));
  yaw1 = atan2(R12[0],R11[0]);

  transformStamped1.header.stamp = ros::Time::now();
  transformStamped1.header.frame_id = "/rslidar";
  transformStamped1.child_frame_id = "/power2_left";
  transformStamped1.transform.translation.x = invert_index * T1[0];
  transformStamped1.transform.translation.y = invert_index * T1[1];
  transformStamped1.transform.translation.z = invert_index * T1[2];
  q1.setRPY(roll1, pitch1, yaw1);
  transformStamped1.transform.rotation.x = invert_index * q1.x();
  transformStamped1.transform.rotation.y = invert_index * q1.y();
  transformStamped1.transform.rotation.z = invert_index * q1.z();
  transformStamped1.transform.rotation.w = invert_index * q1.w();
  br1_.sendTransform(transformStamped1);

  cout << "x y z yaw pitch roll " 
  	   << transformStamped1.header.frame_id 
  	   <<" to "
  		 <<transformStamped1.child_frame_id <<endl;
  cout << transformStamped1.transform.translation.x << " " 
  		 << transformStamped1.transform.translation.y << " "
  		 << transformStamped1.transform.translation.z << " " 
    	 << yaw1 << " " 
    	 << pitch1 << " " 
    	 << roll1 << endl;


	geometry_msgs::TransformStamped transformStamped2;
	tf2::Quaternion q2;	
	double roll2,pitch2,yaw2;
	invert_index = 1;

	roll2 = atan2(R23[1],R23[2]);
  pitch2 = atan2(-R23[0],hypot(R23[1],R23[2]));
  yaw2 = atan2(R22[0],R21[0]);


  transformStamped2.header.stamp = ros::Time::now();
  transformStamped2.header.frame_id = "/power2_left";
  transformStamped2.child_frame_id = "/plane";
  transformStamped2.transform.translation.x = invert_index * T2[0]/1000;
  transformStamped2.transform.translation.y = invert_index * T2[1]/1000;
  transformStamped2.transform.translation.z = invert_index * T2[2]/1000;
  q2.setRPY(roll2, pitch2, yaw2);
  transformStamped2.transform.rotation.x = invert_index * q2.x();
  transformStamped2.transform.rotation.y = invert_index * q2.y();
  transformStamped2.transform.rotation.z = invert_index * q2.z();
  transformStamped2.transform.rotation.w = invert_index * q2.w();
  br2_.sendTransform(transformStamped2);

  cout << "x y z yaw pitch roll " 
  	   << transformStamped2.header.frame_id 
  	   <<" to "
  		 <<transformStamped2.child_frame_id <<endl;
  cout << transformStamped2.transform.translation.x << " " 
  		 << transformStamped2.transform.translation.y << " "
  		 << transformStamped2.transform.translation.z << " " 
    	 << yaw2 << " " 
    	 << pitch2 << " " 
    	 << roll2 << endl;



	geometry_msgs::TransformStamped transformStamped3;
	tf2::Quaternion q3;	
	double roll3,pitch3,yaw3;
	invert_index = -1;

	roll3 = atan2(R33[1],R33[2]);
  pitch3 = atan2(-R33[0],hypot(R33[1],R33[2]));
  yaw3 = atan2(R32[0],R31[0]);


  transformStamped3.header.stamp = ros::Time::now();
  transformStamped3.header.frame_id = "/plane";
  transformStamped3.child_frame_id = "/power2_right";
  transformStamped3.transform.translation.x = invert_index * T3[0]/1000;
  transformStamped3.transform.translation.y = invert_index * T3[1]/1000;
  transformStamped3.transform.translation.z = invert_index * T3[2]/1000;
  q3.setRPY(roll3, pitch3, yaw3);
  transformStamped3.transform.rotation.x = invert_index * q3.x();
  transformStamped3.transform.rotation.y = invert_index * q3.y();
  transformStamped3.transform.rotation.z = invert_index * q3.z();
  transformStamped3.transform.rotation.w = invert_index * q3.w();
  br3_.sendTransform(transformStamped3);

  cout << "x y z yaw pitch roll " 
  	   << transformStamped3.header.frame_id 
  	   <<" to "
  		 <<transformStamped3.child_frame_id <<endl;
  cout << transformStamped3.transform.translation.x << " " 
  		 << transformStamped3.transform.translation.y << " "
  		 << transformStamped3.transform.translation.z << " " 
    	 << yaw3 << " " 
    	 << pitch3 << " " 
    	 << roll3 << endl << endl;




}
