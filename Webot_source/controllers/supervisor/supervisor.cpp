// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "RobotControl.hpp"


int main(int argc, char **argv) {
  // create the Robot instance. 
 
  std::vector<std::vector<point>> seqPoint = getSetPath();
  // seqPoint[0] = {{2,0},{2,10},{6,10},{6,4},{4,4},{4,0},{3,0}};
  // seqPoint[1] = {{0,2},{10,2},{10,7},{0,7},{0,3}};
  // seqPoint[2] = {{0,5},{8,5},{8,9},{0,9},{0,6}};

  std::vector<std::vector<point>> path;

  std::string tmp;
  std::vector<std::string> rbSet;
  for ( int idx = 0;idx<NUM_ROBOT;idx++)
  {
      tmp.assign("robot_");
      tmp.append(std::to_string(idx+1));
      rbSet.push_back(tmp);
  }
      


  for( int i =0; i < NUM_ROBOT ; ++i){
    path.push_back(generatePath(seqPoint[i],10));

  }

 
  
   
  Driver *controller = new Driver(rbSet,path);
  controller->runSim();
  delete controller;
  return 0; 
   
}
