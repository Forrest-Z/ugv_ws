#include "libSonicManager.h"

SonicManager::SonicManager():pn("~")
{
  pn.param<string>("port_name", port_name_, "/dev/ttyUSB0");
  pn.param<int>("baud_rate", baud_rate_, 115200);

  state_pub = n.advertise<std_msgs::Int32>("/sonic_state",1);
  vel_pub = n.advertise<geometry_msgs::Twist> ("/teleop_velocity_smoother/raw_cmd_vel", 1);

  bumper_sub = n.subscribe("/mobile_base/events/bumper",1, &SonicManager::bumper_callback,this);


  emergency_range_ = 20;

  sonic_state_ = -1;
  clock_1 = ros::Time::now();

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
  sonic_state_ = -1;
  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, error_code_);
  if (error_code_) {
    ROS_INFO_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str());
    return;
  }
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  vector<int> sonic_vec;
  vector<int> filtered_vec;

  

  readBaseData(sonic_vec);
  if(findSafeDirection(sonic_vec,filtered_vec)){

    // cout << "=========================="<<endl;
    // cout << "Duration "<< (ros::Time::now() - clock_1).toSec() << endl;
    clock_1 = ros::Time::now();
    // cout << "=========================="<<endl;

    // cout << "avg data: "<< filtered_vec.size()<<endl;
    // for (int i = 0; i < filtered_vec.size(); ++i) {
    //   cout << i << ":"<< filtered_vec[i] << endl;
    // }
    // cout << endl;

    sonic_state_ = makeDecision(filtered_vec);
    sonic_state_ = makeMajorDecision(filtered_vec,sonic_state_);
    if(bumper_state_ == -1) sonic_state_ = -1;
    moveMyself(sonic_state_);  
    sendState();
  }

}

void SonicManager::readBaseData(vector<int>& Output) {
  int char_size = 64;
  vector<char> output_char(char_size);
  vector<int> output_caliber;
  size_t len = port_->read_some(boost::asio::buffer(output_char));
  // cout << "length:"<< len <<endl;
  if(len != 19) return;
  
  int digital = 0;
  int soince_index = 0;
  output_caliber.resize(6,0);

  for (int i = 0; i < len-1; ++i) {
    output_caliber[soince_index] += int(output_char[i] - '0') * pow(10,(2-digital));
    digital++;
    if(digital == 3) {
      digital = 0;
      soince_index++;
    }
  }

  calibrationSonic(output_caliber,Output);

  // cout << "raw data: "<<endl;
  // for (int i = 0; i < Output.size(); ++i) {
  //   cout << i << ":"<< Output[i] << endl;
  // }
  // cout << endl;

}

bool SonicManager::findSafeDirection(vector<int> Input,vector<int>& Output){
  static vector<int> sum_vec(Input.size(),0);
  static vector<int> avg_vec(Input.size(),0);

  static std::queue<int> history_que;
  int queue_size = 5; 

  if(history_que.size() < queue_size*(Input.size()-1)) {
    sum_vec = plusVectors(sum_vec,Input);
    for (int i = 0; i < Input.size(); ++i){
      history_que.push(Input[i]);
    }
    return false; 
  } else {
    sum_vec = plusVectors(sum_vec,Input);
    for (int i = 0; i < Input.size(); ++i){
      history_que.push(Input[i]);
    }
    avg_vec = dividVectors(sum_vec,5);
    Output = avg_vec;

    for (int i = 0; i < Input.size(); ++i){
      sum_vec[i] -= history_que.front();
      history_que.pop();
    }


    return true;
  }
}


int SonicManager::makeMajorDecision(vector<int> range,int state) {
  int output = state;
  if(range[0] > 100 && range[1] > 100) return 0;
  // if(range[0] < 3 && range[1] < 3 && range[2] < 3) {
  //   (range[3] > range[4]) ? output = 3: output = 4;
  //   return output;
  // }
  for (int i = 0; i < range.size(); ++i) {
    if(range[i] < emergency_range_) return -1;
  }

  return output;
}

int SonicManager::makeDecision(vector<int> range) {
  double speed_threshold = 100;
  vector<double> speed_factor = {1,1,0.5,0.2,0.5,0.2};
  vector<double> rotation_factor = {-1,1,1,0.2,-1,-0.2};

  int slowdown_ct = 0;
  int rotation_ct = 0;
  for (int i = 0; i < range.size(); ++i) {
    if(range[i] < speed_threshold * speed_factor[i]) slowdown_ct++;
    if(i > -1) rotation_ct += range[i] * rotation_factor[i];
  }

  // cout << "slowdown_ct: "<< slowdown_ct <<endl;
  // cout << "rotation_ct: "<< rotation_ct <<endl;

  int rotation_threshold = 30;

  if (slowdown_ct < 2 ) {
    if(fabs(rotation_ct) < rotation_threshold) return 0;
    else {
      if(rotation_ct > 0) return 2;
      return 1;
    }
  } else {
    if(rotation_ct > 0) return 3;
    return 4;
  }

}


void SonicManager::moveMyself(int Input) {

  geometry_msgs::Twist cmd_vel;
  if (Input == 0) {
    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = 0;
    vel_pub.publish(cmd_vel);
    cout << "command: ^ " << endl<< endl;
    return;
  }
  else if (Input == 1) {
    cmd_vel.linear.x = 0.1;
    cmd_vel.angular.z = -0.5;
    vel_pub.publish(cmd_vel);
    cout << "command: ^ > " << endl<< endl;
    return;
  }
  else if (Input == 2) {
    cmd_vel.linear.x = 0.1;
    cmd_vel.angular.z = 0.5;
    vel_pub.publish(cmd_vel);
    cout << "command: < ^ " << endl<< endl;
    return;
  }
  else if (Input == 3) {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 1;
    vel_pub.publish(cmd_vel);
    cout << "command: <-- " << endl<< endl;
    return;
  }
  else if (Input == 4) {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = -1;
    vel_pub.publish(cmd_vel);
    cout << "command: --> " << endl<< endl;
    return;
  } else {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    vel_pub.publish(cmd_vel);
    cout << "command: X " << endl<< endl;
    return;
  }

}

void SonicManager::calibrationSonic(vector<int> Input,vector<int>& Output) {
  Output = Input;

  Output[0] = Input[1];
  Output[1] = Input[0];
  Output[2] = Input[5];
  Output[3] = Input[4];
  Output[4] = Input[3];
  Output[5] = Input[2];

}

void SonicManager::sendState() {
  std_msgs::Int32 output;
  output.data = sonic_state_;
  (sonic_state_ == -1) ? output.data = 1 : output.data = 0;

  state_pub.publish(output);

}
