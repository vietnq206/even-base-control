// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "RobotControl.hpp"


int main(int argc, char **argv) {
  // create the Robot instance. 
  int numrobot = 3;
  std::vector<std::vector<point>> seqPoint = getSetPath();
  // seqPoint[0] = {{2,0},{2,10},{6,10},{6,4},{4,4},{4,0},{3,0}};
  // seqPoint[1] = {{0,2},{10,2},{10,7},{0,7},{0,3}};
  // seqPoint[2] = {{0,5},{8,5},{8,9},{0,9},{0,6}};

  std::vector<std::vector<point>> path;


  for( int i =0; i < numrobot ; ++i){
    path.push_back(generatePath(seqPoint[i],10));

  }

 
  std::vector<std::string> rbSet = {"robot_1","robot_2","robot_3"};
   
  Driver *controller = new Driver(rbSet,path);
  controller->runSim();
  delete controller;
  return 0; 
   
}
