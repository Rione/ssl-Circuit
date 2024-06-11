#include "wheelTestMode.hpp"

void WheelTestMode::before() {
    robot->led1 = 1;
    timer.reset();
}   

void WheelTestMode::after() {
    robot->led1 = 0;
   
}

void WheelTestMode::loop(){
    if(timer.read_ms() > 1000){
        timer.reset();
        robot->led1 = !robot->led1;
    }
    
}