// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "RobotControl.hpp"





int main(int argc, char **argv) {
  // create the Robot instance.
   
  
  std::vector<std::string> rbSet = {"robot_1","robot_2"};
   
  Driver *controller = new Driver(rbSet);
  controller->test();
  delete controller;
  return 0; 
   
}
