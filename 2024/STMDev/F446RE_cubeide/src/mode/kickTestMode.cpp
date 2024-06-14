#include "kickTestMode.hpp"

void KickTestMode::before() {
    robot->led1 = 1;
    timer.reset();
}   

void KickTestMode::after() {
    robot->led1 = 0;
   
}

void KickTestMode::loop(){
    if(timer.read_ms() > 100){
        timer.reset();
        robot->led1 = !robot->led1;
    }
    
}