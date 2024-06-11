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

typedef struct {
    union{
        struct{
            uint8_t mode : 6;
            bool emergencyStop : 1;          
            bool parity : 1;
        };
        uint8_t data;
    }status;

    bool modePrev = 0;

} UIModeSwitch_t;

UIModeSwitch_t uiModeSwitchData;

void xiaoRecvSerial(){
    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 1;  // データのサイズ
    static bool headerReceived = false;  // ヘッダを受信したかどうか
    static uint8_t index = 0;            // 受信したデータのインデックスカウンター
    static uint8_t data[dataSize] = {0}; // 受信したデータ

    while(robot.serial4.available()){
        // 1バイト読み込み
        uint8_t receivedByte = robot.serial4.read();
        // printf("received %d\n ", receivedByte);

        if (!headerReceived) {
            index = 0;
            if (receivedByte == HEADER) {
                // ヘッダを受信したらデータの受信を開始
                headerReceived = true; // ヘッダを受信したフラグを立てる
                // printf("header received %d\n ", receivedByte);
            } else {
                headerReceived = false; // ヘッダではないのでフラグをリセット
                // printf("error: Header is not received %d\n", receivedByte);
            }
        } else { // ヘッダを受信した後の処理
            if (index < dataSize) {
                // データ受信
                data[index] = receivedByte;
                // printf("data[%d]: %d\n", index, data[index]);
                index++;

                if (index == dataSize) {
                    // データ受信完了
                    uiModeSwitchData.status.data = data[0];

                    headerReceived = false; // 次のヘッダを待つ準備をする
                    index = 0;              // インデックスをリセット

                }
            }
        }
    }
}

void ModeChange(UIModeSwitch_t *uiModeSwitchData){
    if(uiModeSwitchData->status.mode != uiModeSwitchData->modePrev){
        currentMode->after();
        currentMode = modes[uiModeSwitchData->status.mode];
        currentMode->before();
    
        uiModeSwitchData->modePrev = uiModeSwitchData->status.mode;
    }
}

Timer t;
void ModeChange_timer(UIModeSwitch_t *uiModeSwitchData){
    if(t.read_ms() > 10000){
        currentMode->after();

        uiModeSwitchData->status.mode = (uiModeSwitchData->status.mode + 1) % mode_qty;
        currentMode = modes[uiModeSwitchData->status.mode];
        currentMode->before();

        t.reset();
    }
}


void setup() {
    robot.hardwareInit();
    t.reset();
    
}

void main_app() {

    while (1) {
        // xiaoRecvSerial();
        ModeChange_timer(&uiModeSwitchData);
        currentMode->loop();

                        
    }
}