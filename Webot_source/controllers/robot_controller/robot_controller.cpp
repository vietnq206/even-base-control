// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include <webots/InertialUnit.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <math.h>

// All the webots classes are defined in the "webots" namespace
using namespace std;
using namespace webots; 
#define MAX_SPEED 3


#define TIME_STEP 32
#define Dwth 135
#define Dwth_wall 150

// Robot constants for odometry
#define WHEEL_RADIUS 0.0205 // in m
#define AXLE_LENGTH 0.052 // in m
#define STEPS_ROT 1000 // 1000 steps per rotatation
#define PI 3.141592654
#define PI2 1.570796326



static const double maxSpeed = 10.0;

class Slave : public Robot {
public:
  Slave();
  void run();
  void forward();
  void turnLeft();
  void turnRight();
  void stop();
  void robotOrientation();
private:
  enum Mode { STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN };
  enum Orient {EAST, WEST, NORTH, SOUTH};
  static double boundSpeed(double speed);

   
  Mode mode;
  Orient orient;
  Receiver *receiver; 
  Motor *motors[2];
  GPS *gp;
  Gyro *gr;
  InertialUnit *iu;

};

Slave::Slave() {
  gp = this->getGPS("gps");
  gr = this->getGyro("gyro");
  iu = this->getInertialUnit("imu");
  gp->enable(TIME_STEP);
  gr->enable(TIME_STEP);
  iu->enable(TIME_STEP); 


  mode = AVOID_OBSTACLES;

   
  receiver = getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(0);

  motors[0] = getMotor("left wheel motor");
  motors[1] = getMotor("right wheel motor");

  motors[0]->setPosition(std::numeric_limits<double>::infinity());
  motors[1]->setPosition(std::numeric_limits<double>::infinity());

  motors[0]->setVelocity(0.0);
  motors[1]->setVelocity(0.0);
  
  robotOrientation();
}


void Slave::turnLeft(){
  const double* currIMU = iu->getRollPitchYaw();
  double targetYall = currIMU[2] + PI2;
  if ( targetYall > PI)
    targetYall = fmodf(targetYall,PI) - PI;

  motors[0]->setVelocity(-MAX_SPEED);
  motors[1]->setVelocity(MAX_SPEED);
  while(this->step(TIME_STEP) != -1){

    currIMU = iu->getRollPitchYaw();
    std::cout<<"X ="<<currIMU[0]<<"Y= "<<currIMU[1]<<" Z = "<<currIMU[2]<<std::endl;
    
    // std::cout<<"target : "<<abs(currIMU[2]-targetYall)<<std::endl;
    
    if (abs(currIMU[2]-targetYall)<0.05)
      {
        stop();
        break;
      }
  }

  if(orient == NORTH) orient = WEST;
  else if (orient == WEST) orient = SOUTH;
  else if (orient == SOUTH) orient = EAST;
  else orient = NORTH;


}
void Slave::robotOrientation(){
  const double* currIMU = iu->getRollPitchYaw(); 

  double differentAngle = 2*PI;
  if ( abs(currIMU[2]-0) < differentAngle)
  {
    orient = NORTH;
    differentAngle = abs(currIMU[2]-0);
  }
  if ( abs(currIMU[2]-PI2) < differentAngle)
  {
    orient = WEST;
    differentAngle = abs(currIMU[2]-PI2);
  }

  if ( abs(currIMU[2]+PI2) < differentAngle)
  {
    orient = EAST;
    differentAngle = abs(currIMU[2]+PI2);
  }
  if ( abs(currIMU[2]- PI) < differentAngle && abs(currIMU[2]+PI)< differentAngle)
  {
    orient = SOUTH;
  }
}

void Slave::turnRight(){
  const double* currIMU = iu->getRollPitchYaw();
  double targetYall = currIMU[2] - PI2;
  if ( targetYall < -PI)
    targetYall = fmodf(targetYall,PI) + PI;

  motors[0]->setVelocity(MAX_SPEED);
  motors[1]->setVelocity(-MAX_SPEED);
  while(this->step(TIME_STEP) != -1){

    currIMU = iu->getRollPitchYaw();
    std::cout<<"X ="<<currIMU[0]<<"Y= "<<currIMU[1]<<" Z = "<<currIMU[2]<<std::endl;
    
    std::cout<<"target : "<<abs(currIMU[2]-targetYall)<<std::endl;
    
    if (abs(currIMU[2]-targetYall)<0.05)
      {
        stop();
        break;
      }
  }
 if(orient == NORTH) orient = EAST;
  else if (orient == EAST) orient = SOUTH;
  else if (orient == SOUTH) orient = WEST;
  else orient = NORTH;

}


void Slave::stop(){
  motors[0]->setVelocity(0);
  motors[1]->setVelocity(0);
}


double Slave::boundSpeed(double speed) {
  return std::min(maxSpeed, std::max(-maxSpeed, speed));
}

void Slave::forward(){ 
  double startGPS[3];
  const double* currGPS = gp->getValues();
  bool reachTarget = false;
  motors[0]->setVelocity(MAX_SPEED); 
  motors[1]->setVelocity(MAX_SPEED);
  robotOrientation();

  if ( this->step(TIME_STEP) != -1){
    for ( int i =0; i < 3 ; i++){startGPS[i] = currGPS[i];}
  }


  while(this->step(TIME_STEP) != -1 && !reachTarget)
  {
    std::cout<<"Start :"<<startGPS[2]<<std::endl;
    std::cout<<"Current :"<<currGPS[2]<<std::endl;
    currGPS = gp->getValues();
    switch (orient)
      {
      case NORTH:
        if ( abs(currGPS[2] - startGPS[2]) > 0.1 ) reachTarget = true;
        break;
      case SOUTH:
        if ( abs(currGPS[2] - startGPS[2]) > 0.1) reachTarget = true;
        break;
      case WEST:
        if ( abs(currGPS[0] - startGPS[0]) > 0.1) reachTarget = true;
        break;
      case EAST:
        if ( abs(currGPS[0] - startGPS[0]) > 0.1) reachTarget = true;
        break;
      
      default:
        break;
      }
  }
  stop();

 
}
void Slave::run() {
  // main loop
  std::cout<<"gagag :"<<receiver->getQueueLength() <<std::endl;

  while (this->step(TIME_STEP) != -1) {
        forward();
    turnLeft();
    forward();
    turnRight();
    // // Read sensors, particularly the order of the supervisor
    // if (receiver->getQueueLength() > 0) {
    //   string message((const char *)receiver->getData());
    //   receiver->nextPacket();

    //   cout << "I should " << AnsiCodes::RED_FOREGROUND << message << AnsiCodes::RESET << "!" << endl;

    //   if (message.compare("avoid obstacles") == 0)
    //     mode = AVOID_OBSTACLES;
    //   else if (message.compare("move forward") == 0)
    //     mode = MOVE_FORWARD;
    //   else if (message.compare("stop") == 0)
    //     mode = STOP;
    //   else if (message.compare("turn") == 0)
    //     mode = TURN;
    // }
    // double delta = 2;
    // double speeds[2] = {0.0, 0.0};

    // // send actuators commands according to the mode
    // switch (mode) {
    //   case AVOID_OBSTACLES:
    //     speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
    //     speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
    //     break;
    //   case MOVE_FORWARD:
    //     speeds[0] = maxSpeed;
    //     speeds[1] = maxSpeed;
    //     break;
    //   case TURN:
    //     speeds[0] = maxSpeed / 2.0;
    //     speeds[1] = -maxSpeed / 2.0;
    //     break;
    //   default:
    //     break;
    // }
    // motors[0]->setVelocity(speeds[0]);
    // motors[1]->setVelocity(speeds[1]);
  }
}












int main(int argc, char **argv) {

    Slave *controller = new Slave();
  controller->run();
  delete controller;
  return 0;

}
