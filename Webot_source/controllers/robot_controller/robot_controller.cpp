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
#define MAX_SPEED 6.28


#define TIME_STEP 32
#define Dwth 135
#define Dwth_wall 150

// Robot constants for odometry
#define WHEEL_RADIUS 0.0205 // in m
#define AXLE_LENGTH 0.052 // in m
#define STEPS_ROT 1000 // 1000 steps per rotatation
#define PI 3.141592654
#define PI2 1.570796326



// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

static const double maxSpeed = 10.0;

class Slave : public Robot {
public:
  Slave();
  void run();
  void forward();
  void getWheelDisplacements(double *dispLeftW, double *dispRightW, double del_enLeftW, double del_enRightW);
  void turnLeft();
private:
  enum Mode { STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN };

  static double boundSpeed(double speed);

  int timeStep;
  Mode mode;
  Receiver *receiver;
  Camera *camera;
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
  timeStep = 32;
  mode = AVOID_OBSTACLES;
  camera = getCamera("camera");
  camera->enable(4 * TIME_STEP);
  receiver = getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(0);
  motors[0] = getMotor("left wheel motor");
  motors[1] = getMotor("right wheel motor");
  motors[0]->setPosition(std::numeric_limits<double>::infinity());
  motors[1]->setPosition(std::numeric_limits<double>::infinity());
  motors[0]->setVelocity(0.0);
  motors[1]->setVelocity(0.0);
 
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
    
    if (abs(currIMU[2]-targetYall)<0.01)
      {
        motors[0]->setVelocity(MAX_SPEED);
        //motors[1]->setVelocity(0);
        break;
      }
      


  }


}

void Slave::getWheelDisplacements(double *dispLeftW, double *dispRightW, double del_enLeftW, double del_enRightW) {

  // compute displacement of left wheel in meters
  *dispLeftW = del_enLeftW / STEPS_ROT * 2 * PI * WHEEL_RADIUS; 
  // compute displacement of right wheel in meters
  *dispRightW = del_enRightW / STEPS_ROT * 2 * PI * WHEEL_RADIUS; 
}

double Slave::boundSpeed(double speed) {
  return std::min(maxSpeed, std::max(-maxSpeed, speed));
}

void Slave::forward(){ 

  motors[0]->setVelocity(MAX_SPEED);
  const double* tmp;
  motors[1]->setVelocity(-MAX_SPEED);
  while (this->step(TIME_STEP) != -1)
  {
    tmp = iu->getRollPitchYaw();
    // std::cout<<"X ="<<gr->getValues()[0]<<"Y= "<<gr->getValues()[1]<<" Z = "<<gr->getValues()[2]<<std::endl;
    std::cout<<"X ="<<tmp[0]<<"Y= "<<tmp[1]<<" Z = "<<tmp[2]<<std::endl;
  }

}
void Slave::run() {
  // main loop
  while (step(TIME_STEP) != -1) {
    // Read sensors, particularly the order of the supervisor
    if (receiver->getQueueLength() > 0) {
      string message((const char *)receiver->getData());
      receiver->nextPacket();

      cout << "I should " << AnsiCodes::RED_FOREGROUND << message << AnsiCodes::RESET << "!" << endl;

      if (message.compare("avoid obstacles") == 0)
        mode = AVOID_OBSTACLES;
      else if (message.compare("move forward") == 0)
        mode = MOVE_FORWARD;
      else if (message.compare("stop") == 0)
        mode = STOP;
      else if (message.compare("turn") == 0)
        mode = TURN;
    }
    double delta = 2;
    double speeds[2] = {0.0, 0.0};

    // send actuators commands according to the mode
    switch (mode) {
      case AVOID_OBSTACLES:
        speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
        speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
        break;
      case MOVE_FORWARD:
        speeds[0] = maxSpeed;
        speeds[1] = maxSpeed;
        break;
      case TURN:
        speeds[0] = maxSpeed / 2.0;
        speeds[1] = -maxSpeed / 2.0;
        break;
      default:
        break;
    }
    motors[0]->setVelocity(speeds[0]);
    motors[1]->setVelocity(speeds[1]);
  }
}












int main(int argc, char **argv) {

    Slave *controller = new Slave();
  controller->turnLeft();
  delete controller;
  return 0;



  // create the Robot instance.
  Robot *robot = new Robot();
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity( MAX_SPEED); 
  rightMotor->setVelocity(MAX_SPEED);
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  GPS *gp;
  gp = robot->getGPS("gps");
  gp->enable(TIME_STEP);
  std::cout<<"HELLLP"<<std::endl;
   
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
      std::cout<<"X ="<<gp->getValues()[0]<<"Y= "<<gp->getValues()[1]<<" Z = "<<gp->getValues()[2]<<std::endl;
    
    
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
