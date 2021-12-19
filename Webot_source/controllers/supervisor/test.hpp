
#include "RobotControl.hpp"
#define TIME_STEP 32
#define SIZE_X 10
#define SIZE_Y 10

using namespace webots;

class Driver : public Supervisor {
public:
  Driver(std::vector<std::string> rbSet);
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
  Field *translationField , *translationField2;
  Field *motor1;
  Keyboard *keyboard;
  double x;
  double z;
  double translation[3];
  std::vector<RobotControl>  robots; 

  std::vector<std::vector<int>> mapRegister;
};
