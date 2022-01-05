// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "RobotControl.hpp"

std::vector<point> generatePath(std::vector<point> setPoint){

    std::vector<point> path;
    point prev_p;
    int itvX, itvZ;
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
     path.push_back(setPoint[setPoint.size()-1]);
    
    return path;
}
int main(int argc, char **argv) {
  // create the Robot instance. 
  std::vector<point> seqPointPath1 = {{2,0},{2,10},{16,10},{16,19},{18,19},{18,7},{4,7},{4,0}};
  std::vector<point> seqPointPath2 = {{0,2},{8,2},{8,15},{19,15},{19,17},{4,17},{4,3},{0,3}};
  std::vector<point> seqPointPath3 = {{0,5},{12,5},{12,14},{19,14},{19,9},{0,9}};



  std::vector<point> path1 = generatePath(seqPointPath1);
  std::vector<point> path2 = generatePath(seqPointPath2);
  std::vector<point> path3 = generatePath(seqPointPath3);

 
  std::vector<std::string> rbSet = {"robot_1","robot_2","robot_3"};
   
  Driver *controller = new Driver(rbSet,{path1,path2,path3});
  controller->runSim();
  delete controller;
  return 0; 
   
}
