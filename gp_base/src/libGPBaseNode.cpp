#include "libGPBaseNode.h"

GPBase::GPBase():pn("~")
{
  pn.param<string>("port_name", port_name_, "/dev/ttyACM0"  );
  pn.param<int>("baud_rate", baud_rate_, 115200);

  odom_pub = n.advertise<nav_msgs::Odometry>("/wheel_odom", 1);
  battery_pub = n.advertise<geometry_msgs::Point32>("/battery_info", 1);
  cmd_pub = n.advertise<geometry_msgs::Twist>("/gp_base_cmd", 1);

  cmd_sub = n.subscribe("/husky_velocity_controller/cmd_vel",1, &GPBase::cmd_callback,this);

  isCmdUpdate_ = false;
  isGetFirstData_ = false;

  base_odom_.pose.pose.position.x = 0;
  base_odom_.pose.pose.position.y = 0;
  base_odom_.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(0);
}

GPBase::~GPBase(){
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();  
}

void GPBase::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);

  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, error_code_);
  if (error_code_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str());
    return;
  }
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }

  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();   
}

void GPBase::Mission() {
  // if(!port_->is_open()){
  //   cout << "Reopen port" << endl;
  //   port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  //   port_->open(port_name_, error_code_);
  //   if (error_code_) {
  //     ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str());
  //     sleep(1);   
  //     return;
  //   }
  //   port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  // }
  
  sendBaseData();
  readBaseData();
  // return;

  // if (port_) {
  //   port_->cancel();
  //   port_->close();
  //   port_.reset();
  // }
  // io_service_.stop();
  // io_service_.reset();
}

void GPBase::debugFunction() {
}


void GPBase::sendBaseData() {
  double linear_speed = cmd_vel_.linear.x;
  double angular_speed = cmd_vel_.angular.z;
  double speed_left = (2 * linear_speed - WHELL_BASE_M * angular_speed)/2;
  double speed_right = (2 * linear_speed + WHELL_BASE_M * angular_speed)/2;

  if(!isCmdUpdate_){
    speed_left = 0;
    speed_right = 0;
  }
  isCmdUpdate_ = false;
  int rpm_left = ms2rpm(speed_left);
  int rpm_right = ms2rpm(speed_right);

  if(linear_speed != 0 || angular_speed != 0){
    cout << "Send" << endl;
    cout << "Linear  Speed : " << linear_speed << endl;
    cout << "Angular Speed : " << angular_speed << endl;
    // cout << "Speed   Left  : " << speed_left << endl;
    // cout << "Speed   Right : " << speed_right << endl;    
  }

  geometry_msgs::Twist gp_base_cmd_vel;
  gp_base_cmd_vel.linear.x = speed_left;
  gp_base_cmd_vel.linear.y = speed_right;

  cmd_pub.publish(gp_base_cmd_vel);


  vector<unsigned char> motor_left = num2hex(rpm_left);
  vector<unsigned char> motor_right = num2hex(rpm_right);

  vector<unsigned char>output = {0xa5,0xb0};
  vector<unsigned char>status = {0x05,0x3f};
  vector<unsigned char>check  = {0x00,0x00};
  vector<unsigned char>footer = {0x5a,0x0d};

  output.insert(output.end(), status.begin(), status.end());
  output.insert(output.end(), motor_left.begin(), motor_left.end());
  output.insert(output.end(), motor_right.begin(), motor_right.end());
  output.insert(output.end(), check.begin(), check.end());
  output.insert(output.end(), footer.begin(), footer.end());

  boost::asio::write(*port_.get(),boost::asio::buffer(output),boost::asio::transfer_all(),error_code_);
  //boost::asio::async_write(*port_.get(),boost::asio::buffer(output),boost::asio::transfer_all(),
    //boost::bind(&GPBase::async_write_handler,this,_1,_2));
  // printf("send ");
  // for (int i = 0; i < output.size(); ++i) {
  //   printf("%x ",output[i]);
  // }
  // printf("\n");
}

void GPBase::readBaseData() {
  int char_size = 100; //38;
  vector<unsigned char> output(char_size);
  size_t len = port_->read_some(boost::asio::buffer(output),error_code_);
  // port_->async_read_some(boost::asio::buffer(output),boost::bind(&GPBase::read_some_handler,this,_1,port_));
  // port_->async_read_some(boost::asio::buffer(output),read_some_handler);
  // if (error_code_) {
  //   ROS_INFO_STREAM("error read some " << port_name_ << ", e=" << error_code_.message().c_str()); 

  //   port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  //   port_->open(port_name_, error_code_);
  //   if (error_code_) {
  //     ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str());
  //     sleep(1);   
  //     return;
  //   }
  //   port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    
  //   return;
  // }
  // printf("read ");
  // for (int i = 0; i < output.size(); ++i) {
  //   printf("%x ",output[i]);
  // }
  // printf("\n");
  cout << "read size :" << len << endl;
  cout << "Base upload raw :" ;
  for(int i = 0;i < len;++i) {
  printf("%d : %x ",i,output[i]);
  }
  cout << endl;
  
  if(!checkDataHead(output)) return;
  
  geometry_msgs::Point32 msg_battery;
  float battery_full_voltage = 24;
  float battery_low_voltage  = 20;
  float standard_current = 0.5;
  float standard_time_min = 200;
  float battery_unit = 100/(battery_full_voltage - battery_low_voltage);

  vector<unsigned char> motor_left = {output[22],output[23]};
  vector<unsigned char> motor_right = {output[24],output[25]};

  int rpm_left = static_cast<int>(hex2num(motor_left));
  int rpm_right = static_cast<int>(hex2num(motor_right));

  if(rpm_left >= MAX_16BIT/2) rpm_left -= MAX_16BIT;
  if(rpm_right >= MAX_16BIT/2) rpm_right -= MAX_16BIT;

  double ms_left = rpm2ms(rpm_left);
  double ms_right = rpm2ms(rpm_right);

  double linear_speed = -(ms_right + ms_left)/2;
  double angular_speed = (ms_right - ms_left)/WHELL_BASE_M; 

  if(linear_speed != 0 || angular_speed != 0){
     // cout << "rpm_left : " << rpm_left << endl;
     // cout << "rpm_right : " << rpm_right << endl;  
     cout << "Read" << endl;
     cout << "linear_speed : " << linear_speed << endl;
     cout << "angular_speed : " << angular_speed << endl;    
   }
  base_twist_.linear.x = linear_speed;
  base_twist_.angular.z = angular_speed;

  vector<unsigned char> voltage = {output[26],output[27]};
  vector<unsigned char> current = {output[28],output[29]};
  msg_battery.x = static_cast<float>(hex2num(voltage))/100;
  msg_battery.y = static_cast<float>(hex2num(current))/10;
  float battery_percent = 100 - (battery_unit * (battery_full_voltage - msg_battery.x));
  float current_time_left = standard_time_min * battery_percent * (standard_current/msg_battery.y) * 0.01;
  if(current_time_left < 0) current_time_left = -1;
  addBatteryHistory(current_time_left);
  msg_battery.z = checkBatteryAverage();
  battery_pub.publish(msg_battery);
  publishOdom();
}


int GPBase::hex2num(vector<unsigned char> Input){
  int Output;
  int part_1 = int(Input[0]);
  int part_2 = int(Input[1]);

  std::stringstream ss_1;
  std::stringstream ss_2;
  ss_1 << std::hex << part_1;
  ss_2 << std::hex << part_2;

  string output_1_str = fillZero(ss_1.str(),2);
  string output_2_str = fillZero(ss_2.str(),2);

  string output = output_2_str + output_1_str;

  int output_int = stoi(output,nullptr,16);
  Output = output_int;

  return Output;
}


bool GPBase::checkDataHead(vector<unsigned char> input) {
  vector<unsigned char>header = {0xa5,0xb0};
  // printf("Header : %x,%x\n",header[0],header[1]);
  // printf("Input  : %x,%x\n",input[0],input[1]);
  if(input[0] == header[0] && input[1] == header[1]) 
  {
    return true;
  }
  return false;
}


void GPBase::publishOdom() {
  double x = base_odom_.pose.pose.position.x;
  double y = base_odom_.pose.pose.position.y;
  double th = tf::getYaw(base_odom_.pose.pose.orientation);

  double vx = base_twist_.linear.x;
  double vy = 0;
  double vth = base_twist_.angular.z * 0.45;

  //vth = 0;
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  // cout << "Odometry" << endl;
  // cout << "current : " << current_time_.toSec() << endl;
  // cout << "last : " << last_time_.toSec() << endl;
  // cout << "dt : " << dt << endl;
  // cout << "delta_x : " << delta_x << endl;
  // cout << "delta_y : " << delta_y << endl;
  // cout << "delta_th : " << delta_th << endl;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = 0;
  odom_trans.transform.translation.y = 0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  // odom_broadcaster.sendTransform(odom_trans);

  odom_quat = tf::createQuaternionMsgFromYaw(th);
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



vector<unsigned char> GPBase::num2hex(int Input){
  vector<unsigned char> Output;
  if(Input >= 0) {
    std::stringstream ss;
    ss << std::hex << Input;
    string output_str = ss.str();
    for (int i = 0; i < output_str.size()-4; ++i) {
      output_str = "0" + output_str;
    }

    string part_1 = output_str.substr(0,2);
    string part_2 = output_str.substr(2,2);


    int part_1_int = stoi(part_1,nullptr,16);
    int part_2_int = stoi(part_2,nullptr,16);

    unsigned char part_1_char = part_1_int;
    unsigned char part_2_char = part_2_int; 


    Output.push_back(part_2_char);
    Output.push_back(part_1_char);
  } else {
    std::stringstream ss;
    ss << std::hex << abs(Input);
    string output_str = ss.str();
    for (int i = 0; i < output_str.size()-4; ++i) {
      output_str = "0" + output_str;
    }

    int before_int = stoi(output_str,nullptr,16);
    std::bitset<16> input_bin(before_int);
    input_bin = ~input_bin;
    int after_int = stoi(input_bin.to_string(),nullptr,2);
    after_int++;

    std::stringstream ss_after;
    ss_after << std::hex << after_int;
    string output_str_after = ss_after.str();

    for (int i = 0; i < output_str_after.size()-4; ++i) {
      output_str_after = "0" + output_str_after;
    }

    string part_1 = output_str_after.substr(0,2);
    string part_2 = output_str_after.substr(2,2);


    int part_1_int = stoi(part_1,nullptr,16);
    int part_2_int = stoi(part_2,nullptr,16);

    unsigned char part_1_char = part_1_int;
    unsigned char part_2_char = part_2_int; 

    Output.push_back(part_2_char);
    Output.push_back(part_1_char);
  }
  return Output;
}
