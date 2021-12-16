
#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>

#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>



using namespace webots;
#define TIME_STEP 32
#define SIZE_X 10
#define SIZE_Y 10

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes


struct point{
    int locX;
    int locZ;
};





class RobotControl{
private:
Node* robot;
Field *translationField; 
bool activeMode;
std::vector<point> pathJobs;
int robotID;

public:
  RobotControl(Node* rb, int rbID);
  ~RobotControl();
  const double* getLocation();
  void setLocation(double* location);
  bool askMove(point askLoc, std::vector<std::vector<int>> mapRegister);
};


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
