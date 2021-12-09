// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
#define TIME_STEP 32
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
   
  Supervisor *sv = new Supervisor();
  Node *robot = sv->getFromDef("e-puck");
  // get the time step of the current world.
 
  Field *pos = robot->getField("translation");
  Field *rot = robot->getField("rotation");
  
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  double loc[3] = {0,0.01,0};
  double orien[4] = {1,0,0,0};
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (sv->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    loc[2] = loc[2] + 0.01;
    pos->setSFVec3f(loc);
    rot->setSFRotation(orien);
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete sv;
  return 0;
}
