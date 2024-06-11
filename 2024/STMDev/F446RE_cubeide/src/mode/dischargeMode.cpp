#include "dischargeMode.hpp"

void DischargeMode::before() {
    robot->led1 = 1;
    timer.reset();
}   

void DischargeMode::after() {
    robot->led1 = 0;
   
}

void DischargeMode::loop(){
    if(timer.read_ms() > 1000){
        timer.reset();
        robot->led1 = !robot->led1;
    }
    
}