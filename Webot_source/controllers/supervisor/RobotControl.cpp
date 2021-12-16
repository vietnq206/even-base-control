#include "RobotControl.hpp"


RobotControl::RobotControl(Node* rb, int rbID){ 
  robot = rb;
  robotID = rbID;
  // this->robotID = rbID;
  translationField = robot->getField("translation"); 
}
const double* RobotControl::getLocation(){ 
   return translationField->getSFVec3f();
 }
void RobotControl::setLocation(double* location){
  translationField->setSFVec3f(location);
}

RobotControl::~RobotControl(){ 
}
bool RobotControl::askMove(point askLoc, std::vector<std::vector<int>> mapRegister){
    if ( mapRegister[askLoc.locX][askLoc.locZ] == robotID) return true;
    else return false;
}



