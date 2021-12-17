#include <fstream>
#include <iostream>
#include "system_protocol.pb.h"

int main() {
    protocal::RobotEvent comm;
    comm.set_robotid(2);
    comm.set_eventsent("haha");  
    std::string ofs;

    comm.SerializeToString(&ofs);

    std::cout<<ofs<<std::endl;
}