#include <MotorDriver.hpp>

MotorDriver::MotorDriver(CANBus *_canBus, BNO055 *bno) : canBus(_canBus) {
}

void MotorDriver::init() {
}

void MotorDriver::setVelocityFF(float velX, float velY, float velZ) {
}

void MotorDriver::setVelocity(float velX, float velY, float velZ) {
}

void MotorDriver::setPositionOfMotor(float pos1, float pos2, float pos3, float pos4) {
}

void MotorDriver::setPIDGain(float kp, float ki, float kd) {
}