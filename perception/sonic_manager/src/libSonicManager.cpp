#include "libSonicManager.h"

SonicManager::SonicManager():pn("~")
{
  pn.param<string>("port_name", port_name_, "/dev/ttyUSB0");
  pn.param<int>("baud_rate", baud_rate_, 115200);

  state_pub = n.advertise<std_msgs::Int32>("/sonic_state",1);

  sonic_state_ = 0;
}

SonicManager::~SonicManager(){
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();  
}

void SonicManager::Manager() {

  ros::Rate loop_rate(ROS_RATE_HZ);
  
  while (ros::ok()) {
    Mission();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void SonicManager::Mission() {
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, error_code_);
  if (error_code_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str());
    return;
  }
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  
  readBaseData();
  sendState();  
}

void SonicManager::readBaseData() {
  int char_size = 16;
  sonic_state_ = 0;
  vector<char> output(char_size);
  size_t len = port_->read_some(boost::asio::buffer(output));

  if(len != 3) return;
  sonic_state_ = (int)(output[0] - '0');
}

void SonicManager::sendState() {
  std_msgs::Int32 output;
  output.data = sonic_state_;
  state_pub.publish(output);

}
