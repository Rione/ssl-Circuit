#ifndef __Robot__
#define __Robot__

#include "main.h"

#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "BufferedSerial.hpp"
#include "DMAStream.h"
#include "CAN.hpp"
#include "BNO055.hpp"
#include "Timer.hpp"

#include "MotorDriver.hpp"

#include "adc.h"

typedef struct {
    //     /*
    //     0. ヘッダ0xFFを受信
    //     1. velX[LSB]
    //     2. velX[MSB]
    //     3. velY[LSB]
    //     4. velY[MSB]
    //     5. velAngler[LSB]
    //     6. velAngler[MSB]
    //     7. dribblePower[0~100]
    //     8. kickerPowerStraight[0~100]
    //     9. kickerPowerChip[0~100]
    //     10.status
    //       . bit0: emergencyStop
    //       . bit1: doDirectKick
    //       . bit2: doDirectChipKick
    //       . bit3: reserved
    //       . bit4: reserved
    //       . bit5: reserved
    //       . bit6: reserved
    //       . bit7: parity
    //      */

    // Infomation RaspberryPi→STM32
    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t vel;
    } velX;
    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t vel;
    } velY;
    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t vel;
    } velAngler;

    uint8_t dribblePower;
    uint8_t kickerPowerStraight;
    uint8_t kickerPowerChip;
    union {
        struct {
            bool emergencyStop : 1;
            bool doDirectKick : 1;
            bool doDirectChipKick : 1;
            bool reserved0 : 1;
            bool reserved2 : 1;
            bool reserved3 : 1;
            bool reserved4 : 1;
            bool parity : 1;
        };
        uint8_t data;
    } status;

    // Infomation STM32→RaspberryPi
    uint8_t photoSensorValue;
    uint8_t isHoldBall;
    uint8_t batteryVoltage;
} RobotInfo;

class Robot {
  public:
    Robot();

    // values
    RobotInfo info;

    // peripherals
    CANBus can = CANBus(&hcan1, 0);

    DigitalOut led0 = DigitalOut(LED0_GPIO_Port, LED0_Pin);
    DigitalOut led1 = DigitalOut(LED1_GPIO_Port, LED1_Pin);
    DigitalOut led2 = DigitalOut(LED2_GPIO_Port, LED2_Pin);
    PwmSingleOut ledH = PwmSingleOut(&htim1, TIM_CHANNEL_1);

    BufferedSerial serial1 = BufferedSerial(&huart1, 128); // pc
    BufferedSerial serial4 = BufferedSerial(&huart4, 128); // xiao
    BufferedSerial serial5 = BufferedSerial(&huart5, 256); // RasPi
    BNO055 bno = BNO055(&hi2c1);

    MotorDriver motorDriver = MotorDriver(&can, &bno);

    void hardwareInit();
    void rasRecvSerial();
    void rasSendSerial(RobotInfo &info);

    inline __attribute__((always_inline)) void heartBeat() {
        static int i = 0;
        i++;
        ledH.write(MyMath::sinDeg(int(i / 5)) / 2 + 0.5);
    }
};

#endif