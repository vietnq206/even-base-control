#include<iostream>
#include<vector>
#include<unistd.h>
#include<stdlib.h> 


#define SIZEX 10
#define SIZEY 10



int get_location(std::vector<int> loc, std::vector<std::vector<int>> setLoc)
{
    for( int i = 0; i<setLoc.size();i++)
    {
        if( loc[0] == setLoc[i][0] && loc[1] == setLoc[i][1])
            return i;
    }
    return -1;

}


void setup(std::vector<std::vector<int>> location){

int num_robot = location.size();

std::vector<int> tmp;

for ( int i=0;i<SIZEX;i++)
{
    for ( int j=0;j<SIZEY;j++)
    {
        tmp = {i,j};

        if ( get_location(tmp, location) != -1)
            std::cout<<"| "<<get_location(tmp, location)<<" |";
        else
            std::cout<<"|   |";


    }
    std::cout<<std::endl;
}

sleep(1);


}







int main(){
    std::vector<int> s ={1,2};
    std::vector<std::vector<int>> set = {{0,2},{2,5},{5,6}};
    
    
    int i = 0;
    while( i < 9)
    {
        setup(set);
        set[0][0]++;

        i++;
        system("CLS");
    }
    setup(set) ; 
    return 0;





}