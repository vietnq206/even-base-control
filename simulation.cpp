#include<iostream>
#include<vector>
#include<unistd.h>
#include<stdlib.h> 
#include<math.h>
#include<fstream>

#define SIZEX 10
#define SIZEY 10



int get_location(std::vector<int> loc, std::vector<std::vector<int>> setLoc){
    for( int i = 0; i<setLoc.size();i++){
        if( loc[0] == setLoc[i][0] && loc[1] == setLoc[i][1])
            return i;
    }
    return -1;

}


void setup(std::vector<std::vector<int>> &location){

    int num_robot = location.size();
    std::vector<int> tmp;
    for ( int i=0;i<SIZEX;i++){
        for ( int j=0;j<SIZEY;j++){
            tmp = {i,j};
            if ( get_location(tmp, location) != -1)
                std::cout<<"| "<<get_location(tmp, location)<<" |";
            else
                std::cout<<"|   |";
        }
        std::cout<<std::endl;
        for ( int j=0;j<SIZEY;j++){
            std::cout<<"_____";
        }
        std::cout<<std::endl;
    }
    sleep(1);
    }

inline int calNorm1(std::vector<int> a,std::vector<int> b){   return (abs(a[0] - b[0]) + abs(a[1]-b[1])); }

void releaseRegister(std::vector<std::vector<int>> setLoc,std::vector<std::vector<int>> &mapRegister){
    for ( int row =0; row<SIZEX ;++row){
        for ( int col = 0; col<SIZEY ;++col ){
            if ( mapRegister[row][col] != -1){
                if (calNorm1(setLoc[mapRegister[row][col]],{row,col}))
                    mapRegister[row][col] = -1;
            }
        }
    }
}

void robotRegister(std::vector<std::vector<int>> askLoc,std::vector<std::vector<int>> &mapRegister){
    for ( int rb = 0; rb < askLoc.size(); ++rb){
        if (mapRegister[askLoc[rb][0]][askLoc[rb][1]] == -1)  mapRegister[askLoc[rb][0]][askLoc[rb][1]] = rb;
    }
}

bool askMove(int robotName,std::vector<int> currLoc, std::vector<int> askLoc, std::vector<std::vector<int>> mapRegister){
    if ( mapRegister[askLoc[0]][askLoc[1]] == robotName) return true;
    else return false;
}


int main(){
    std::vector<int> elmRegistor(SIZEX,-1);
    std::vector<std::vector<int>> mapRegistor(SIZEY,elmRegistor);

    std::vector<int> s ={1,2};
    std::vector<std::vector<int>> set = {{0,2},{2,0}};

    //Robot1 sequence of moving
    std::vector<std::vector<int>> set1 = {{0,2},{1,2},{2,2},{3,2},{3,3},{3,4},{3,5},{3,6},{3,7},{3,8},{3,9}};
    
    //Robot2 sequence of moving
    std::vector<std::vector<int>> set2 = {{2,0},{2,1},{2,2},{2,3},{3,3},{4,3},{5,3},{6,3},{7,3},{8,3},{9,3}};
   
    //register at the beginning
    robotRegister(set,mapRegistor);
    int i = 0;
    int numRobot = 2;
    int r1Indx = 1;
    int r2Indx = 1;
    while( 1)
    {
        //display initial location of 2 robots    
        setup(set);
        
        // Show the register of the map
        for ( int i=0;i<SIZEX;i++){
            for ( int j=0;j<SIZEY;j++){
                std::cout<<mapRegistor[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout << "Press Enter to Continue";
        std::cin.ignore();


        // set of requesting moves
        std::vector<std::vector<int>> setAsk ;
        
        setAsk.push_back(set1[r1Indx]);
        setAsk.push_back(set2[r2Indx]);

        //Robot try to registor the next location
        robotRegister(setAsk,mapRegistor);

        //Robot try to get the next move
        if (askMove(0,set1[r1Indx-1],set1[r1Indx],mapRegistor)&& r1Indx<set1.size()-1) r1Indx++;
        if (askMove(1,set2[r2Indx-1],set2[r2Indx],mapRegistor)&&r2Indx<set2.size()-1) r2Indx++;

        set[0] = set1[r1Indx-1];
        set[1] = set2[r2Indx-1];

        //Rekease the register if robot move out
        releaseRegister(set,mapRegistor);
        system("CLS");
    }
    setup(set) ; 
    return 0;





}