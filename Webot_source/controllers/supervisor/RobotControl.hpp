
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
  void run();
  void robotRegister(std::vector<point> askLoc);
  void releaseRegister(std::vector<point> setLocRobot );
  void test();
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

void Driver::releaseRegister(std::vector<point> setLocRobot ){
    for ( int row =0; row<SIZE_X ;++row){
        for ( int col = 0; col<SIZE_Y ;++col ){
            if ( mapRegister[row][col] != -1){
                if (calNorm1(setLocRobot[mapRegister[row][col]],point{row,col}))
                    mapRegister[row][col] = -1;
            }
        }
    }
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

void Driver::robotRegister(std::vector<point> askLoc){
    for ( int rb = 0; rb < askLoc.size(); ++rb){
        if (mapRegister[askLoc[rb].locX][askLoc[rb].locZ] == -1)  mapRegister[askLoc[rb].locX][askLoc[rb].locZ] = rb;
    }
}



void Driver::test(){
  double LocR1[3] = {0.8,0,-1};
  double LocR2[3] = {1,0,-0.8};

  std::vector<std::string> ackAskRobot;

  for ( int i = 0; i< numRobot; ++i )
  {
    ackAskRobot.push_back("0");
  }

  // robots[0].setLocation(LocR1);
  // robots[1].setLocation(LocR2);
  std::vector<point> set = {pathRobot[0][0],pathRobot[1][0]};
      //Robot1 sequence of moving
    std::vector<point> set1 = pathRobot[0];
    //Robot2 sequence of moving
    std::vector<point> set2 = pathRobot[1];
  robotRegister(set);

  std::string message("");
  std::string messIn("");
  emitter->setChannel(0);
  int r1Indx = 1;
  int r2Indx = 1;
  int t1 = 0;
  int t2 = 0;
  while(step(timeStep) != -1)
    {
      




      // std::cout<<"PRESEND"<<std::endl;
      if (!message.empty() ) { 
      // std::cout<<"SENDING"<<std::endl;
      emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
      }
      message.assign("#");
      message.append(std::to_string(t1));
      
      message.append("1#11#13"); 
      t1 == 0 ? t1 = 1: t1 = 0;
      
      // if(t1 == 0 ) t1 = 1;
      // else t1 = 0;

      if (receiver->getQueueLength() > 0) {
      messIn = ((const char *)receiver->getData());
      std::cout<<"REEIVED : "<<messIn<<std::endl;
      receiver->nextPacket();
    }




      int k = keyboard->getKey();
      if (k == 'N')
      {
        // set of requesting moves
        std::vector<point> setAsk ;
        
        setAsk.push_back(set1[r1Indx]);
        setAsk.push_back(set2[r2Indx]);

        //Robot try to registor the next location
        robotRegister(setAsk);

        //Robot try to get the next move
        if (robots[0].askMove(set1[r1Indx],mapRegister)&& r1Indx<set1.size()-1) r1Indx++;
        if (robots[1].askMove(set2[r2Indx],mapRegister)&&r2Indx<set2.size()-1) r2Indx++;

        set[0] = set1[r1Indx-1];
        set[1] = set2[r2Indx-1];
        LocR1[0] = 1 - set[0].locX*0.1;
        LocR1[2] = -1 + set[0].locZ*0.1;

        LocR2[0] = 1 - set[1].locX*0.1;
        LocR2[2] = -1 + set[1].locZ*0.1;

        robots[0].setLocation(LocR1);
        robots[1].setLocation(LocR2);



        //Rekease the register if robot move out
        releaseRegister(set); 
      }

    }

}

void Driver::run() {
  std::string previous_message("");
  std::string message("");

  displayHelp();

  // main loop
  while (step(timeStep) != -1) {

    // Read sensors; update message according to the pressed keyboard key
    int k = keyboard->getKey();
    switch (k) {
      case 'C':
        std::cout<<"channel 0"<<std::endl;
        emitter->setChannel(0);
        break;
      case 'V':
        std::cout<<"channel 1"<<std::endl;
        emitter->setChannel(1);
        break;
      case 'A':
        
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
        const double *translationValues = robots[0].getLocation();
        std::cout << "ROBOT1 is located at (" << translationValues[0] << "," << translationValues[2] << ")" << std::endl;
        break;
      }
      case 'R':
        std::cout << "Teleport ROBOT1 at (" << x << "," << z << ")" << std::endl;
       // translationField->setSFVec3f(translation);
        robots[0].setLocation(translation);
        break;
      default:
        message.clear();
    }

    // send actuators commands; send a new message through the emitter device
    if (!message.empty() && message.compare(previous_message)) {
      previous_message.assign(message);
      std::cout << "Please, " << message.c_str() << std::endl;
      emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
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
