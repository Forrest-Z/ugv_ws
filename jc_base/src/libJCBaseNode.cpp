#include "libJCBaseNode.h"

JCBase::JCBase():pn("~")
{
  pn.param<string>("port_name", port_name_, "/dev/ttyUSB0");
  pn.param<int>("baud_rate", baud_rate_, 115200);

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

  cmd_sub = n.subscribe("/husky_velocity_controller/cmd_vel",1, &JCBase::cmd_callback,this);

  sleep(1);

  isCmdUpdate_ = false;
  isGetFirstData_ = false;

  base_odom_.pose.pose.position.x = 0;
  base_odom_.pose.pose.position.y = 0;
  base_odom_.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(0);
}

JCBase::~JCBase(){
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();  
}

void JCBase::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void JCBase::Mission() {
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, error_code_);
  if (error_code_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str());
    return;
  }
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  sendBaseData();

  readBaseData();

  publishOdom();
  
}

void JCBase::sendBaseData() {
  double linear_speed = cmd_vel_.linear.x;
  double angular_speed = cmd_vel_.angular.z;

  double speed_1 = (2 * linear_speed - WHELL_BASE_M * angular_speed)/2;
  double speed_2 = (2 * linear_speed + WHELL_BASE_M * angular_speed)/2;

  speed_1 *= -1;
  speed_2 *= -1;

  if(!isCmdUpdate_){
    speed_1 = 0;
    speed_2 = 0;
  }
  isCmdUpdate_ = false;

  int num1 = ms2rpm(speed_1) - ms2rpm(speed_2);
  int num2 = (ms2rpm(speed_1) + ms2rpm(speed_2))/2;

  int num1_index = 0;
  int num2_index = 0;

  if(num1 < 0) num1_index = 1;
  if(num2 < 0) num2_index = 1;

  vector<char> output{0x21,0x6d,0x20};
  vector<char> output_footer{0x0d,0x0a};
  vector<char> space{0x20};

  vector<char> num1_vector = num2hex(num1);
  vector<char> num2_vector = num2hex(num2);

  output.insert(output.end(), num1_vector.begin(), num1_vector.end());
  output.insert(output.end(), space.begin(), space.end());
  output.insert(output.end(), num2_vector.begin(), num2_vector.end());
  output.insert(output.end(), output_footer.begin(), output_footer.end());

  vector<char> check_odom{0x3f,0x53,0x0d,0x0a};
  output.insert(output.end(), check_odom.begin(), check_odom.end());

  boost::asio::write(*port_.get(),
    boost::asio::buffer(output,32),
    error_code_);
}

void JCBase::readBaseData() {
  int char_size = 64;
  double linear_speed = 0;
  double angular_speed = 0;
  vector<char> output(char_size);

  size_t len = port_->read_some(boost::asio::buffer(output));

  if(checkDataHead(output)){
    if(!isGetFirstData_){
      last_time_ = ros::Time::now();
      isGetFirstData_ = true;
    }
    current_time_ = ros::Time::now();

    int speed_1 = 0;
    int speed_2 = 0;
    int index_1 = 1;
    int index_2 = 1;

    int mid_index = 0;
    int end_index = 0;
    for (int i = 5; i < char_size; ++i){
      if(int(output[i]) <= 0) break;
      if(int(output[i]) == 58){
        mid_index = i;
      }
      if(int(output[i]) == 13){
        end_index = i;
        break;
      }
    }

    for (int i = 5; i < char_size; ++i){
      if(int(output[i]) <= 0 || int(output[i]) == 13) break;
      if(i == mid_index) continue;

      if(i < mid_index){
        if(int(output[i]) == 45){
          index_1 = -1;
          continue;
        }
        int power_1 = mid_index - i - 1;
        speed_1 += pow(10,power_1) * (int(output[i])-48);
      }

      if(i > mid_index){
        if(int(output[i]) == 45){
          index_2 = -1;
          continue;
        }
        int power_2 = end_index - i - 1;
        speed_2 += pow(10,power_2) * (int(output[i])-48);
      }
    }

    speed_1 *= index_1 * -1;
    speed_2 *= index_2 * 1;

    linear_speed = (rpm2ms(speed_1) + rpm2ms(speed_2))/2;
    angular_speed = (rpm2ms(speed_2) - rpm2ms(speed_1))/WHELL_BASE_M;  
  }

  base_twist_.linear.x = linear_speed;
  base_twist_.angular.z = angular_speed;

}


bool JCBase::checkDataHead(vector<char>& input) {
  int odom_begin = -1;
  for (int i = 0; i < input.size()-1; ++i) {
    if(input[i] == 63 && input[i+1] == 83){
      odom_begin = i;
      break;
    }
  }
  if(odom_begin < 0) return false;

  auto first = input.begin() + odom_begin;
  auto last = input.end();
  vector<char> temp(first,last);

  input = temp;

  if(input[0] != 63) return false;
  if(input[1] != 83) return false;
  if(input[2] != 13) return false;
  if(input[3] != 83) return false;
  if(input[4] != 61) return false;
  return true;
}


void JCBase::publishOdom() {
  double x = base_odom_.pose.pose.position.x;
  double y = base_odom_.pose.pose.position.y;
  double th = tf::getYaw(base_odom_.pose.pose.orientation);

  double vx = base_twist_.linear.x;
  double vy = 0;
  double vth = base_twist_.angular.z * 0.45;

  vth = 0;

  double dt = (current_time_ - last_time_).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = 0;
  odom_trans.transform.translation.y = 0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);


  base_odom_.header.stamp = current_time_;
  base_odom_.header.frame_id = "odom";

  base_odom_.pose.pose.position.x = x;
  base_odom_.pose.pose.position.y = y;
  base_odom_.pose.pose.position.z = 0.0;
  base_odom_.pose.pose.orientation = odom_quat;

  base_odom_.child_frame_id = "base_link";
  base_odom_.twist.twist.linear.x = vx;
  base_odom_.twist.twist.linear.y = vy;
  base_odom_.twist.twist.angular.z = vth;

  odom_pub.publish(base_odom_);

  last_time_ = current_time_;
}
