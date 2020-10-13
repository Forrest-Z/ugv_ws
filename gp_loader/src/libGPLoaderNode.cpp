#include "libGPLoaderNode.h"

GPLoader::GPLoader():pn("~")
{
  pn.param<string>("port_name", port_name_, "/dev/ttyACM1");
  pn.param<int>("baud_rate", baud_rate_, 115200);

  cmd_sub = n.subscribe("/loader_command1",1, &GPLoader::cmd_callback,this);
  level_sub = n.subscribe("/level_command",1, &GPLoader::level_callback,this);
}

GPLoader::~GPLoader(){
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();  
}

void GPLoader::Manager() {

  cout << "Loader Node For Started" << endl;

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

void GPLoader::Mission() {
  // debugFunction();
  // return;

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
  // readBaseData();

  // return;

  // if (port_) {
  //   port_->cancel();
  //   port_->close();
  //   port_.reset();
  // }
  // io_service_.stop();
  // io_service_.reset();
}

void GPLoader::debugFunction() {
}


void GPLoader::sendBaseData() {
  int ramp_straight_up  = 249;

  vector<unsigned char>output = {0xa5,0xb0};
  vector<unsigned char>status = {0x03,0x00};
  vector<unsigned char>check  = {0x00,0x00};
  vector<unsigned char>footer = {0x5a,0x0d};

  int speed_value = loader_gear_;
  int level_value = ramp_straight_up - loader_level_;

    cout << "Speed Value : " << loader_gear_ << endl;
    cout << "Level Value : " << loader_level_ << endl;   

  vector<unsigned char>motor_speed = num2hex(speed_value);
  vector<unsigned char>ramp_level = num2hex(level_value);

  output.insert(output.end(), status.begin(), status.end());
  output.insert(output.end(), motor_speed.begin(), motor_speed.end());
  output.insert(output.end(), ramp_level.begin(), ramp_level.end());
  output.insert(output.end(), check.begin(), check.end());
  output.insert(output.end(), footer.begin(), footer.end());

  boost::asio::write(*port_.get(),boost::asio::buffer(output),boost::asio::transfer_all(),error_code_);
  // boost::asio::async_write(*port_.get(),boost::asio::buffer(output),boost::asio::transfer_all(),
  //   boost::bind(&GPLoader::async_write_handler,this,_1,_2));
  // printf("send ");
  // for (int i = 0; i < output.size(); ++i) {
  //   printf("%x ",output[i]);
  // }
  // printf("\n");
}

void GPLoader::readBaseData() {
}


int GPLoader::hex2num(vector<unsigned char> Input){
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


bool GPLoader::checkDataHead(vector<unsigned char> input) {
  vector<unsigned char>header = {0xa5,0xb0};
  // printf("Header : %x,%x\n",header[0],header[1]);
  // printf("Input  : %x,%x\n",input[0],input[1]);
  if(input[0] == header[0] && input[1] == header[1]) 
  {
    return true;
  }
  return false;
}



vector<unsigned char> GPLoader::num2hex(int Input){
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
