// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "RobotControl.hpp"

std::vector<point> generatePath(std::vector<point> setPoint){

    std::vector<point> path;
    point tmp,start,end;
    end = setPoint[0];
    
    for (auto elm : setPoint)
    {
    start = end;
    end = elm;    
    if (start.locX != end.locX){
      for ( int i=start.locX;i<end.locX;++i)
        {
            tmp.locX = i;
            tmp.locZ = start.locZ;
            path.push_back(tmp);
        }       
    }
    else
    {
      for ( int i=start.locZ;i<end.locZ;++i)
        {
            tmp.locZ = i;
            tmp.locX = start.locX;
            path.push_back(tmp);
        }       
    }
    
    
    
    }
     path.push_back(setPoint[setPoint.size()-1]);
    
    return path;
}


int main(int argc, char **argv) {
  // create the Robot instance. 
  std::vector<point> seqPointPath1 = {{2,0},{2,10},{16,10},{16,19}};
  std::vector<point> seqPointPath2 = {{0,2},{8,2},{8,15},{19,15}};



  std::vector<point> path1 = generatePath(seqPointPath1);
  std::vector<point> path2 = generatePath(seqPointPath2);


  std::vector<std::string> rbSet = {"robot_1","robot_2"};
   
  Driver *controller = new Driver(rbSet,{path1,path2});
  controller->test();
  delete controller;
  return 0; 
   
}
