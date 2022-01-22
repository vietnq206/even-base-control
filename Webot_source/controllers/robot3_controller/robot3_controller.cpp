

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
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include <webots/InertialUnit.hpp>

#include "/home/mice/PWR/Event-Base/even-base-control/Webot_source/controllers/lib.hpp"


// All the webots classes are defined in the "webots" namespace
using namespace std;
using namespace webots; 
 
#define Dwth 135
#define Dwth_wall 150

// Robot constants for odometry
#define WHEEL_RADIUS 0.0205 // in m
#define AXLE_LENGTH 0.052 // in m
#define STEPS_ROT 1000 // 1000 steps per rotatation
#define PI 3.141592654
#define PI2 1.570796326

static const double maxSpeed = 10.0;

 

inline int ch2int(char c) { return int(c)-48; }
 

class Slave : public Robot {
public:
  Slave();
  void run();
  void forward();
  void turnLeft();
  void turnRight();
  void stop();
  void robotOrientation();
  point getVectorOrient();
private:
  enum Mode {STOP, GO};
  enum Orient {EAST, WEST, NORTH, SOUTH}; 

   
  Mode mode;
  Orient orient;
  Emitter *emitter;
  Receiver *receiver; 
  Motor *motors[2];
  GPS *gp;
  Gyro *gr;
  InertialUnit *iu;
  int robotNum;
  int ackAskMove;
  std::queue<int> exeCommand;
  std::vector<point> path;
};

Slave::Slave() {
  gp = this->getGPS("gps");
  gr = this->getGyro("gyro");
  iu = this->getInertialUnit("imu");
  gp->enable(TIME_STEP);
  gr->enable(TIME_STEP);
  iu->enable(TIME_STEP); 


  mode = STOP;
  orient = NORTH;
  std::vector<std::vector<point>> seqPoint = getSetPath();
  path = generatePath(seqPoint[2],10);
   
  emitter = getEmitter("emitter");
  emitter->setChannel(1);

  receiver = getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(0);

  motors[0] = getMotor("left wheel motor");
  motors[1] = getMotor("right wheel motor");

  motors[0]->setPosition(std::numeric_limits<double>::infinity());
  motors[1]->setPosition(std::numeric_limits<double>::infinity());

  motors[0]->setVelocity(0.0);
  motors[1]->setVelocity(0.0);
   
  robotNum = 3;
  ackAskMove = 0;
}

point Slave::getVectorOrient(){
  if (orient == NORTH) return {0,1};
  if ( orient == WEST) return {-1,0};
  if ( orient == SOUTH) return {0,-1};
  return {1,0};
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
    // std::cout<<"X ="<<currIMU[0]<<"Y= "<<currIMU[1]<<" Z = "<<currIMU[2]<<std::endl;
    
    
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
    orient = SOUTH;
    differentAngle = abs(currIMU[2]-0);
  }
  if ( abs(currIMU[2]-PI2) < differentAngle)
  {
    orient = EAST;
    differentAngle = abs(currIMU[2]-PI2);
  }

  if ( abs(currIMU[2]+PI2) < differentAngle)
  {
    orient = WEST;
    differentAngle = abs(currIMU[2]+PI2);
  }
  if ( abs(currIMU[2]- PI) < differentAngle || abs(currIMU[2]+PI)< differentAngle)
  {
    orient = NORTH;
  }


  // std::cout<<" Orienta: "<<orient<<std::endl;
  // std::cout<<" Angle: "<<currIMU[2]<<std::endl;
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
void Slave::forward(){ 
  double startGPS[3];
  const double* currGPS = gp->getValues();
  bool reachTarget = false;
  motors[0]->setVelocity(MAX_SPEED); 
  motors[1]->setVelocity(MAX_SPEED);
  robotOrientation();
  std::string messOut("");
  bool signalSent = false;

  if ( this->step(TIME_STEP) != -1){
    for ( int i =0; i < 3 ; i++){startGPS[i] = currGPS[i];}
  }
  messOut.assign(std::to_string(robotNum));
  messOut.append(std::to_string(ackAskMove));
  
      

    while(this->step(TIME_STEP) != -1 && !reachTarget)
  {
    // std::cout<<"Start :"<<startGPS[2]<<std::endl;
    // std::cout<<"Current :"<<currGPS[2]<<std::endl;
        currGPS = gp->getValues();  
    switch (orient)
      {
      case NORTH:
        if ( abs(currGPS[2] - startGPS[2]) > 0.1 ) reachTarget = true;
        if ( abs(currGPS[2] - startGPS[2]) > 0.05 && !signalSent) 
        {
            // emitter->send(messOut.c_str(), (int)strlen(messOut.c_str()) + 1);
            messOut = "";
            signalSent = true;
        }
        break;
      case SOUTH:
        if ( abs(currGPS[2] - startGPS[2]) > 0.1) reachTarget = true;
        if ( abs(currGPS[2] - startGPS[2]) > 0.05 && !signalSent) 
                {
            // emitter->send(messOut.c_str(), (int)strlen(messOut.c_str()) + 1);
            messOut = "";
            signalSent = true;
        }
        break;
      case WEST:   
       if ( abs(currGPS[0] - startGPS[0]) > 0.1) reachTarget = true;
        if ( abs(currGPS[0] - startGPS[0]) > 0.05 && !signalSent) 
                {
            // emitter->send(messOut.c_str(), (int)strlen(messOut.c_str()) + 1);
            messOut = "";
            signalSent = true;
        }       
        break;
      case EAST:
        if ( abs(currGPS[0] - startGPS[0]) > 0.1) reachTarget = true;
        if ( abs(currGPS[0] - startGPS[0]) > 0.05 && !signalSent) 
        {
            // emitter->send(messOut.c_str(), (int)strlen(messOut.c_str()) + 1);
            messOut = "";
            signalSent = true;
        }  
      default:
        break;
      }
  }
  std::cout<<" Ack R3 - in forward: "<<ackAskMove<<std::endl;
  stop();

 
}

void Slave::run() {
  // int ackAskMove = 0;
  std::string prev_mess = "";
  std::string message;
  std::string messOut("");
  int indexPath = 1;
  float decision;
  int locMessRb = (robotNum-1)*3;
  point vecDesign;
  point vecOritent = getVectorOrient();
  for (auto elm: path){
      std::cout<<"{"<<elm.locX<<","<<elm.locZ<<"},";
    }
    std::cout<<std::endl;

  messOut.assign(std::to_string(robotNum));
  messOut.append(std::to_string(ackAskMove));



  while (this->step(TIME_STEP) != -1) {
      std::cout<<" Ack R3: "<<ackAskMove<<std::endl;
     if (!messOut.empty() ) { 
      // std::cout<<"SENDING"<<std::endl;
      emitter->send(messOut.c_str(), (int)strlen(messOut.c_str()) + 1);
      messOut = "";
      }
    while (receiver->getQueueLength() > 0) {
      message = ((const char *)receiver->getData());
      if(message.compare(locMessRb,2,prev_mess))
      {
        exeCommand.push(ch2int(message[locMessRb+2]));
        prev_mess = message.substr(locMessRb,2);
      }
      receiver->nextPacket();
      std::cout<<"Received: "<<message<<std::endl;
    }
    if (!exeCommand.empty() && indexPath <= path.size())
    {
      if(exeCommand.front() == 1)
      {
        vecDesign.locX = path[indexPath].locX-path[indexPath-1].locX;
        vecDesign.locZ = path[indexPath].locZ-path[indexPath-1].locZ;
        decision = -vecOritent.locX*vecDesign.locZ + vecOritent.locZ*vecDesign.locX;
        // std::cout<<"Decision of "<<indexPath<<" is :"<<decision<<std::endl;
        if(decision==0) 
        {
          std::cout<<"R3: Taking point: X "<<path[indexPath].locX<<" and Y "<<path[indexPath].locZ<<std::endl;
          
          ackAskMove == 0 ? ackAskMove = 1: ackAskMove = 0;
      std::cout<<" Ack R3 - 2: "<<ackAskMove<<std::endl;

          forward();

      std::cout<<" Ack R3 - after: "<<ackAskMove<<std::endl;

          exeCommand.pop();
          indexPath++;
          
          messOut.assign(std::to_string(robotNum));
          messOut.append(std::to_string(ackAskMove));
        }
        else if( decision > 0) turnRight();
        else turnLeft();
      }
      else {
        exeCommand.pop();
      }
      stop();
      vecOritent = getVectorOrient();   
    }
    else stop();
  } 
    
}




int main(int argc, char **argv) {

    Slave *controller = new Slave();
  controller->run();
  delete controller;
  return 0;

}
