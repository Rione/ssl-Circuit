#include "dribbleTestMode.hpp"

void DribbleTestMode::before() {
    robot->led1 = 1;
    timer.reset();
}   

void DribbleTestMode::after() {
    robot->led1 = 0;
   
}

void DribbleTestMode::loop(){
    if(timer.read_ms() > 100){
        timer.reset();
        robot->led1 = !robot->led1;
    }
    
}