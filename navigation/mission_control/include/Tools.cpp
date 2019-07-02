#include <Tools.h>

Tools::Tools()
{
	cout<<"build Tools"<<endl;
}

Tools::~Tools(){
}

void Tools::recordLog(string input,int level) {
  string state;
  string event;
  string color;
  int schedule_selected;
  int log_gap_sec;

  switch (level) {
    case 0: {
      state = "Initialization";
      event = input;
      schedule_selected = 0;
      log_gap_sec = -1;
      color = WHITE;
    break;
    }
    
    case 1: {
      state = "Infomation";
      event = input;
      schedule_selected = 0;
      log_gap_sec = -1;
      color = WHITE;
    break;
    }

    case 2: {
      state = "Controller State";
      event = input;
      schedule_selected = 1;
      log_gap_sec = 1;
      color = GREEN;
    break;
    }

    case 3: {
      state = "WARNNING";
      event = input;
      schedule_selected = 2;
      log_gap_sec = 1;
      color = YELLOW;
    break;
    }

    case4: {
      state = "ERROR";
      event = input;
      schedule_selected = 3;
      log_gap_sec = -1;
      color = RED;
    break;
    }

    default:
      assert(false);
  }

  cout<<color<<"Node  | "<<ros::this_node::getName()<<endl;
  cout<<color<<"Data  | "<<asctime(std::localtime(&clock));
  cout<<color<<"State | "<<state<<endl;
  cout<<color<<"Event | "<<event<<endl;
}