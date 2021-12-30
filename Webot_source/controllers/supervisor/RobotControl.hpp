
#include <webots/Emitter.hpp>

#include <webots/Receiver.hpp>
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
#define SIZE_X 20
#define SIZE_Y 20

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes


struct point{
    int locX;
    int locZ;
};



inline int ch2int(char c) { return int(c)-48; }

class RobotControl{
private:
Node* robot;
Field *translationField, *rotationField; 
bool activeMode;
std::vector<point> pathJobs;
int robotID;
double tmp[4] = {0,1,0,0};


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
  rotationField = robot->getField("rotation");
}
const double* RobotControl::getLocation(){ 
   return translationField->getSFVec3f();
 }
void RobotControl::setLocation(double* location){
  translationField->setSFVec3f(location);
  rotationField->setSFRotation(tmp);
}

RobotControl::~RobotControl(){ 
}
bool RobotControl::askMove(point askLoc, std::vector<std::vector<int>> mapRegister){
    if ( mapRegister[askLoc.locX][askLoc.locZ] == robotID) return true;
    else return false;
}







class Driver : public Supervisor {
public:
  Driver(std::vector<std::string> rbSet,std::vector<std::vector<point>> path);
  void robotRegister(point askLoc, int rbNum);
  void releaseRegister(point LocRobot);
  void printRegisterMap();
  void runSim();
  int calNorm1(point a,point b){   return (abs(a.locX - b.locX + abs(a.locZ-b.locZ))); };


private:
  static void displayHelp();
  int timeStep;
  int numRobot;
  Emitter *emitter;
  Receiver *receiver; 
  Field *translationField , *translationField2;
  Field *motor1;
  Keyboard *keyboard;
  double x;
  double z;
  double translation[3];
  std::vector<RobotControl>  robots; 
  std::vector<std::vector<int>> mapRegister;
  std::vector<std::vector<point>> pathRobot;
};

void Driver::releaseRegister(point LocRobot ){
    // for ( int row =0; row<SIZE_X ;++row){
    //     for ( int col = 0; col<SIZE_Y ;++col ){
    //         if ( mapRegister[row][col] != -1){
    //             if (calNorm1(setLocRobot[mapRegister[row][col]],point{row,col}))
    //                 mapRegister[row][col] = -1;
    //         }
    //     }
    // }
  mapRegister[LocRobot.locX][LocRobot.locZ] = -1;

}
Driver::Driver(std::vector<std::string> rbSet,std::vector<std::vector<point>> path) {
  timeStep = 128;
  numRobot = rbSet.size();
  x = 0.1f;
  z = 0.3f;
  translation[0] = x;
  translation[1] = 0;
  translation[2] = z;

  pathRobot = path;

  emitter = getEmitter("emitter");
  emitter->setChannel(0);

  receiver = getReceiver("receiver");
  receiver->enable(TIME_STEP);
  receiver->setChannel(1);



  std::vector<int> tmp;
  for (int i =0;i<SIZE_X;++i){
    for (int j =0;j<SIZE_Y;++j)
      tmp.push_back(-1);
    mapRegister.push_back(tmp);
  }
  

  for (int idx =0; idx < numRobot; ++idx){
    robots.push_back({getFromDef(rbSet[idx]),idx}); 
  }

  keyboard = getKeyboard();
  keyboard->enable(timeStep);
}

void Driver::robotRegister(point askLoc, int rbNum){
    // for ( int rb = 0; rb < askLoc.size(); ++rb){
    //     if (mapRegister[askLoc[rb].locX][askLoc[rb].locZ] == -1)  mapRegister[askLoc[rb].locX][askLoc[rb].locZ] = rb;
    // }
  if(mapRegister[askLoc.locX][askLoc.locZ] == -1)
    mapRegister[askLoc.locX][askLoc.locZ] = rbNum;
}

void Driver::printRegisterMap(){
  
      std::cout<<std::endl;
      std::cout<<std::endl;
      for ( int i=0;i<SIZE_X;i++){
            for ( int j=0;j<SIZE_Y;j++){
                std::cout<<mapRegister[i][j]<<" ";
            }
            std::cout<<std::endl;
        }  
}

void Driver::runSim(){

  
  std::vector<char> ackAskRobot;
  std::vector<bool> rbDoneEvent;
  std::vector<int> rbIndex;
  std::vector<int> ackMove;

  for ( int i = 0; i< numRobot; ++i )
  {
    ackAskRobot.push_back('0');
    rbDoneEvent.push_back(false);
    rbIndex.push_back(1);
    ackMove.push_back(0);
  }

  std::string messOut("#11#11#11"); //starting signal 
  std::string messOut_prev("");
  std::string messIn("");

  //Setting channel for emmiter
  emitter->setChannel(0);

  while(step(timeStep) != -1)
    {
      for ( int i = 0; i< numRobot; ++i )
        { 
          rbDoneEvent[i] = false;
        }
      if (!messOut.empty() && messOut.compare(messOut_prev) ) { 
        // std::cout<<"SENDING"<<std::endl;
        std::cout<<"MESS send from Supervisor: "<<messOut<<std::endl;
        emitter->send(messOut.c_str(), (int)strlen(messOut.c_str()) + 1);
        messOut_prev = messOut;
      }
      while(receiver->getQueueLength() > 0) {
        messIn = ((const char *)receiver->getData());
        if(!messIn.empty())
        {
        std::cout<<"Mess into Supervisor : "<<messIn<<std::endl;
          if (ackAskRobot[ch2int(messIn[0])-1] != messIn[1]) 
          {
            // rbDoneEvent[ch2int(messIn[0])-1] ? rbDoneEvent[ch2int(messIn[0])-1] = false : rbDoneEvent[ch2int(messIn[0])-1] = true;
            rbDoneEvent[ch2int(messIn[0])-1] = true; //true mean that the robot done the job and asking for new movement
            ackAskRobot[ch2int(messIn[0])-1] = messIn[1];
          }
        }
        receiver->nextPacket();
      }
      messOut = "";
      for( int i =0; i<numRobot;++i)
      {
        if(rbDoneEvent[i]){
          printRegisterMap();
          releaseRegister(pathRobot[i][rbIndex[i]]);
          rbIndex[i]++;
          robotRegister(pathRobot[i][rbIndex[i]],i);
           if (robots[i].askMove(pathRobot[i][rbIndex[i]],mapRegister))
            {
            //       std::cout<<"Robot "<< i<<std::endl;
            // std::cout<<"HAHAHAH1"<<std::endl;
            ackMove[i]  == 0 ? ackMove[i]  = 1: ackMove[i]  = 0;
            }
          }
          else if ( mapRegister[pathRobot[i][rbIndex[i]].locX][pathRobot[i][rbIndex[i]].locZ] != i)
          {
            robotRegister(pathRobot[i][rbIndex[i]],i);
           if (robots[i].askMove(pathRobot[i][rbIndex[i]],mapRegister))
            {
              // std::cout<<"Robot "<< i<<std::endl;
              // std::cout<<"HAHAHAH2 "<< rbIndex[i]<<std::endl;
              ackMove[i] == 0 ? ackMove[i]  = 1: ackMove[i]  = 0;
            }
          }
        messOut.append("#");
        messOut.append(std::to_string(ackMove[i] ));
        messOut.append("1");
      }

    }

}

void Driver::displayHelp() {
  std::string s("Commands:\n"
           " I for displaying the commands\n"
           " A for avoid obstacles\n"
           " F for move forward\n"
           " S for stop\n"
           " T for turn\n");
  std::cout << s << std::endl;
} 
