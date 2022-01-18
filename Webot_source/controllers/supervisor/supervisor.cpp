// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "RobotControl.hpp"

std::vector<point> generatePath(std::vector<point> setPoint, int freq){

    std::vector<point> path;
    point prev_p;
    int itvX, itvZ;
    
    
    for ( int idx = 0; idx < freq; idx ++)
    
    {
            prev_p = setPoint[0];
      
      for (int i=1;i<setPoint.size();++i)
      {
        setPoint[i].locX == prev_p.locX ? itvX = 0 :  itvX = int((setPoint[i].locX - prev_p.locX)/abs(setPoint[i].locX - prev_p.locX));
        setPoint[i].locZ == prev_p.locZ ? itvZ = 0 :  itvZ = int((setPoint[i].locZ - prev_p.locZ)/abs(setPoint[i].locZ - prev_p.locZ));
          
        while(!(prev_p == setPoint[i]))
          {
            path.push_back(prev_p);
            prev_p.locX += itvX;  
            prev_p.locZ += itvZ;  
  
          }
        prev_p = setPoint[i];  
      }
      //add the last elem
      path.push_back(setPoint[setPoint.size()-1]);
    }
  
    
    return path;
}
int main(int argc, char **argv) {
  // create the Robot instance. 
  int numrobot = 3;
  std::vector<std::vector<point>> seqPoint(numrobot);
  seqPoint[0] = {{2,0},{2,10},{6,10},{6,4},{4,4},{4,0},{3,0}};
  seqPoint[1] = {{0,2},{10,2},{10,7},{0,7},{0,3}};
  seqPoint[2] = {{0,5},{8,5},{8,9},{0,9},{0,6}};

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
