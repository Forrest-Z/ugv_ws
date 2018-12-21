#include "libAisimbaBase.h"

AisimbaBase::AisimbaBase():pn("~")
{
  pn.param<string>("port_name", port_name_, "/dev/ttyUSB0");
  pn.param<int>("baud_rate", baud_rate_, 115200);
  pn.param<int>("control_rate", control_rate_, 10);

  cmd_sub = n.subscribe("/base_cmd_vel",1, &AisimbaBase::cmd_callback,this);

  sleep(1);
}


AisimbaBase::~AisimbaBase(){
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();  
}

void AisimbaBase::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);
  
  while (ros::ok()) {
    Mission();
    //debugFunction();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void AisimbaBase::Mission() {
  // if (port_) {
  //   ROS_ERROR("error : port is already opened...");
  //   return;
  // }
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
    return;
  }
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  sendBaseData();
  
  //send_speed_timer = n.createTimer(ros::Duration(1.0/control_rate_), &AisimbaBase::sendBaseData, this);
}



uint8_t AisimbaBase::movingStateUpdate() {
  double oscillation = 0.1;
  if(cmd_vel_.linear.x >= 0) {
    if (cmd_vel_.linear.x == 0) return STOP; 
    if (cmd_vel_.angular.z > oscillation) return FORWARD_LEFT;
    else if (cmd_vel_.angular.z < -oscillation) return FORWARD_RIGHT;
    else return FORWARD;
  } else {
    if (cmd_vel_.angular.z > oscillation) return BACKWARD_RIGHT;
    else if (cmd_vel_.angular.z < -oscillation) return BACKWARD_LEFT;
    else return BACKWARD;
  }
}


void AisimbaBase::sendBaseData() {
  /*
  -data[2] moving state 
    0x03 forward
    0x04 left-forward
    0x05 right-forward
    0x06 stop
    0x07 backward
    0x08 left-backward
    0x09 right-backward

  -data[4] speed (0-100)
    speed = [(data[4]>>4) * 10 + (data[4]&0x0f)] * 10
    data[4] = 0x10  speed = 100; 
  
  -data[5] data[6] steering (max 35)
    steering = [(data[5] - 0x30)*10 + (data[6]- 0x30]
  */

  double max_speed = 2;
  uint8_t data[8] = {0x55, 0xaa, 0x04, 0x00, 0x00, 0x30, 0x30, 0xbb};
  data[2] = movingStateUpdate();
  data[4] = max_speed/3 * 10 * (fabs(cmd_vel_.linear.x)/0.8);
  int steering = 35 * (fabs(cmd_vel_.angular.z)/0.6);
  data[5] = int(steering/10) + 0x30;
  data[6] = (steering%10) + 0x30;

  // for (int i = 0; i < 8; ++i) {
  //   cout <<" 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(data[i]);
  // }
  // cout<<endl;

  double base_speed = static_cast<int>(data[4]) / 10.00 * 3.00;
  double forward_back;
  double left_right;
  (steering == 0) ? left_right = 1 : left_right = fabs(cmd_vel_.angular.z)/cmd_vel_.angular.z;
  (base_speed == 0) ? forward_back = 1 : forward_back = fabs(cmd_vel_.linear.x)/cmd_vel_.linear.x;

  cout << "Linear Speed " << forward_back * base_speed
    << " Steering Angle " << left_right * steering  << endl;
  

  boost::asio::write(*port_.get(), boost::asio::buffer(data, 8), ec_);

}

void AisimbaBase::debugFunction() {

  double max_speed = 2;
  uint8_t data[8] = {0x55, 0xaa, 0x04, 0x00, 0x00, 0x30, 0x30, 0xbb};
  data[2] = movingStateUpdate();
  data[4] = max_speed/3 * 10 * (fabs(cmd_vel_.linear.x)/0.8);
  int steering = 35 * (fabs(cmd_vel_.angular.z)/0.6);
  data[5] = int(steering/10) + 0x30;
  data[6] = (steering%10) + 0x30;

  // for (int i = 0; i < 8; ++i) {
  //   cout <<" 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(data[i]);
  // }
  // cout<<endl;

  double base_speed = static_cast<int>(data[4]) / 10.00 * 3.00;
  double forward_back;
  double left_right;
  (steering == 0) ? left_right = 1 : left_right = fabs(cmd_vel_.angular.z)/cmd_vel_.angular.z;
  (base_speed == 0) ? forward_back = 1 : forward_back = fabs(cmd_vel_.linear.x)/cmd_vel_.linear.x;

  cout << "Linear Speed " << forward_back * base_speed
    << " Steering Angle " << left_right * steering  << endl;
}