#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU6500.hpp"
#include "MadgwickAHRS.h"
#include "FLASH_EEPROM.hpp"

#include "kickTestMode.hpp"
#include "wheelTestMode.hpp"
#include "dribbleTestMode.hpp"
#include "dischargeMode.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);
KickTestMode kickTestMode('K', "Kick Mode", &robot);
WheelTestMode wheelTestMode('W', "Motor Mode", &robot);
DribbleTestMode dribbleTestMode('D', "Dribble Mode", &robot);
DischargeMode dischargeMode('C', "Discharge Mode", &robot);

MPU6500 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
Flash_EEPROM flash;

MPU6500::acc_t acc;
MPU6500::gyro_t gyro;
MPU6500::xyz_t att;

Madgwick filter;
float frontDeg = 0;

const uint8_t mode_qty = 5;
Mode *modes[mode_qty] = {&mainMode, &kickTestMode, &wheelTestMode, &dribbleTestMode, &dischargeMode};
Mode *currentMode = &mainMode;


void mpuget() {
    if (mpu.isCalibrated() == true) {
        mpu.getAccGyro(&acc, &gyro, false);
        filter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
        att.z = (float)MyMath::gapDegrees180(filter.getYaw(), frontDeg);
    }
}

void TimInterrupt1khz() {
    robot.heartBeat();
    robot.swImu.update();
}

void TimInterrupt4khz() {
    mpuget();
}

void canRxInterrupt(CAN_HandleTypeDef *hcan) {
    if (robot.can.getHcan() == hcan) {
        robot.can.recv(canRecvData);
        robot.led0 = !robot.led0;
        switch (canRecvData.stdId) {
        case 0x123: // フォトセンサの値
            robot.info.photoSensorValue = (uint16_t)(canRecvData.data[0]) | (uint16_t)(canRecvData.data[1]) << 8;
            break;
        default:
            break;
        }
    }
}

void ModeChange(UIModeSwitch_t *_uiModeSwitchData){
    // printf("mode %d modePrev:%d\n", _uiModeSwitchData->status.mode, _uiModeSwitchData->modePrev);
    if(_uiModeSwitchData->status.mode != _uiModeSwitchData->modePrev){
        currentMode->after();
        currentMode = modes[_uiModeSwitchData->status.mode];
        currentMode->before();
    }
    _uiModeSwitchData->modePrev = _uiModeSwitchData->status.mode;
}

void setup() {
    robot.hardwareInit();    
    robot.uiModeSwitchData.status.mode = 0; //mainMode
}

void main_app() {
    currentMode->before();
    currentMode->loop();

    while (1) {

        currentMode->encode(&robot.uiModeSwitchData);
        ModeChange(&robot.uiModeSwitchData);
        currentMode->loop();

        // currentMode->encode();
        // HAL_Delay(1000);

        // currentMode->encode();
        // HAL_Delay(1000);


        //受信
        // if(robot.serial4.available()){
        //     printf("received %d\n ", robot.serial4.read());   
        //     robot.led0 = !robot.led0;    
        // }
        
        //送る
        // robot.serial4.write(130);
        // printf("send %d\n ", 130);
        // HAL_Delay(1000);





                        
    }
}