#include "app.h"

#include "Robot.hpp"
#include "mainMode.hpp"
#include "MPU6500.hpp"
#include "MadgwickAHRS.h"
#include "FLASH_EEPROM.hpp"

Robot robot;
CANBus::CANData canRecvData;

MainMode mainMode('M', "Main Mode", &robot);
TestMode testMode('T', "Test Mode", &robot);

MPU6500 mpu(&hspi2, SPI2_CS0_GPIO_Port, SPI2_CS0_Pin);
Flash_EEPROM flash;

MPU6500::acc_t acc;
MPU6500::gyro_t gyro;
MPU6500::xyz_t att;

Madgwick filter;
float frontDeg = 0;

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
            bool mode : 1;
            bool emergencyStop : 1;
            int mode : 2;
            int _mode : 3;
            bool parity : 1;
        };
        uint8_t data;
    }status;

    uint8_t paramator_1;
    uint8_t paramator_2;
    uint8_t paramator_3;

    bool mode_prev;

} XiaoData;

XiaoData UIInfo;

void xiaoRecvSerial(){
    static const uint8_t HEADER = 0xFF;  // ヘッダ
    static const uint8_t dataSize = 3;  // データのサイズ
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
                    UIInfo.status.data = data[0];
                    UIInfo.paramator_1 = data[1];
                    UIInfo.paramator_2 = data[2];
                    UIInfo.paramator_3 = data[3];

                    headerReceived = false; // 次のヘッダを待つ準備をする
                    index = 0;              // インデックスをリセット

                }
            }
        }
    }
}

void ModeSelect(XiaoData *UIInfo){
    if(){
        
    }else if( == 1){
        
    }
    
}



void setup() {
    robot.hardwareInit();
    robot.led0 = 1;
    
}

void main_app() {

    while (1) {

                
    }
}