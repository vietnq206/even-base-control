// File:          supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>

#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>



// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace std;
using namespace webots;
#define TIME_STEP 32

class Driver : public Supervisor {
public:
  Driver();
  void run();

private:
  static void displayHelp();
  int timeStep;
  Emitter *emitter;
  Field *translationField;
  Keyboard *keyboard;
  double x;
  double z;
  double translation[3];
};


Driver::Driver() {
  timeStep = 128;
  x = 0.1f;
  z = 0.3f;
  translation[0] = x;
  translation[1] = 0;
  translation[2] = z;
  emitter = getEmitter("emitter");
<<<<<<< HEAD
  emitter->setChannel(0);
  Node *robot = getFromDef("robot_1");
=======
  Node *robot = getFromDef("e-puck");
>>>>>>> main
  if (!robot)
    // robot might be NULL if the controller is about to quit
    exit(1);

  translationField = robot->getField("translation");
  keyboard = getKeyboard();
  keyboard->enable(timeStep);
}


void Driver::run() {
  string previous_message("");
  string message("");

  displayHelp();

  // main loop
  while (step(timeStep) != -1) {

    // Read sensors; update message according to the pressed keyboard key
    int k = keyboard->getKey();
    switch (k) {
<<<<<<< HEAD
      case 'C':
        cout<<"channel 0"<<endl;
        emitter->setChannel(0);
        break;
      case 'V':
        cout<<"channel 1"<<endl;
        emitter->setChannel(1);
        break;
      case 'A':
        
=======
      case 'A':
        cout<<"gagag"<<endl;
>>>>>>> main
        message.assign("avoid obstacles");
        break;
      case 'F':
        message.assign("move forward");
        break;
      case 'S':
        message.assign("stop");
        break;
      case 'T':
        message.assign("turn");
        break;
      case 'I':
        displayHelp();
        break;
      case 'G': {
        const double *translationValues = translationField->getSFVec3f();
        cout << "ROBOT1 is located at (" << translationValues[0] << "," << translationValues[2] << ")" << endl;
        break;
      }
      case 'R':
        cout << "Teleport ROBOT1 at (" << x << "," << z << ")" << endl;
        translationField->setSFVec3f(translation);
        break;
      default:
        message.clear();
    }

    // send actuators commands; send a new message through the emitter device
    if (!message.empty() && message.compare(previous_message)) {
      previous_message.assign(message);
      cout << "Please, " << message.c_str() << endl;
      emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
    }
  }
}
void Driver::displayHelp() {
  string s("Commands:\n"
           " I for displaying the commands\n"
           " A for avoid obstacles\n"
           " F for move forward\n"
           " S for stop\n"
           " T for turn\n");
  cout << s << endl;
}


int main(int argc, char **argv) {
  // create the Robot instance.
   
   
  Driver *controller = new Driver();
  controller->run();
  delete controller;
  return 0; 
   
   
   
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
