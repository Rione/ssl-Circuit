#ifndef __Robot__
#define __Robot__

#include "main.h"
#include "config.h"

#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "BufferedSerial.hpp"
#include "DMAStream.h"
#include "CAN.hpp"
#include "Timer.hpp"
#include "Button.hpp"
#include "MotorDriver.hpp"
#include "adc.h"
#include "Median.h"
#include "Average.h"

typedef struct {
    // 0. ヘッダ0xFFを受信
    // 1. velX[LSB]
    // 2. velX[MSB]
    // 3. velY[LSB]
    // 4. velY[MSB]
    // 5. velAngler[LSB]
    // 6. velAngler[MSB]
    // 7. dribblePower[0~100] [uint8_t]
    // 8. kickerPowerStraight[0~100] [uint8_t]
    // 9. kickerPowerChip[0~100] [uint8_t]
    // 10. relativePositionX[LSB]
    // 11. relativePositionX[MSB]
    // 12. relativePositionY[LSB]
    // 13. relativePositionY[MSB]
    // 14. relativeTheta[LSB]
    // 15. relativeTheta[MSB]
    // 16. cameraX [uint8_t]
    // 17. cameraY [uint8_t]
    // 18.status
    //   . bit0: emergencyStop
    //   . bit1: doDirectKick
    //   . bit2: doDirectChipKick
    //   . bit3: reserved
    //   . bit4: reserved
    //   . bit5: reserved
    //   . bit6: isCtrlByRobot (0: RACOON-Ctrl, 1: Robot-local-Ctrl)
    //   . bit7: parity

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
    struct {
        uint8_t straight;
        uint8_t chip;
    } kicker;

    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t pos;
    } relativePositionX;

    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t pos;
    } relativePositionY;

    union {
        struct {
            char L : 8;
            char H : 8;
        };
        int16_t theta;
    } relativeTheta;

    struct {
        uint8_t x;
        uint8_t y;
    } camera;

    union {
        struct {
            bool emergencyStop : 1;
            bool doDirectKick : 1;
            bool doDirectChipKick : 1;
            bool reserved0 : 1;
            bool reserved2 : 1;
            bool reserved3 : 1;
            bool isCtrlByRobot : 1; // (0: RACOON-Ctrl, 1: Robot-local-Ctrl)
            bool parity : 1;
        };
        uint8_t data;
    } status;

    // Infomation STM32→RaspberryPi
    uint8_t isHoldBall;
    uint8_t batteryVoltage;

    // local
    uint16_t photoSensorValue;
    bool isUnderVoltage;
} RobotInfo;


typedef struct {
    union{
        struct{
            uint8_t mode : 6;
            bool emergencyStop : 1;  
            bool parity : 1;        
        };
        uint8_t data;
    }status;

    uint8_t modePrev = 0;

} UIModeSwitch_t;

typedef struct {
    uint8_t parametor1;
    uint8_t paramator2;
    uint8_t paramator3;
} UIModeParametor_t;


class Robot {
  public:
    Robot();

    // values
    RobotInfo info;

    UIModeSwitch_t uiModeSwitchData;

    // peripherals
    CANBus can = CANBus(&hcan1, 0);

    DigitalOut led0 = DigitalOut(LED0_GPIO_Port, LED0_Pin);
    DigitalOut led1 = DigitalOut(LED1_GPIO_Port, LED1_Pin);
    DigitalOut led2 = DigitalOut(LED2_GPIO_Port, LED2_Pin);
    PwmSingleOut ledH = PwmSingleOut(&htim1, TIM_CHANNEL_1);
    Button swImu = Button(IMU_SW_GPIO_Port, IMU_SW_Pin);

    BufferedSerial serial1 = BufferedSerial(&huart1, 128); // pc
    BufferedSerial serial4 = BufferedSerial(&huart4, 128); // xiao
    BufferedSerial serial5 = BufferedSerial(&huart5, 256); // RasPi

    MotorDriver motorDriver = MotorDriver(&can);

    Timer rasSendInterval = Timer();

    void hardwareInit();
    void rasRecvSerial();
    void rasSendSerial(RobotInfo &info, uint16_t interval);
    void getSensors(RobotInfo *info);

    void dribble(uint8_t power);

    void chargeStart();
    void discharge();
    void kickStraight(uint8_t power);
    void kickChip(uint8_t power);

    inline __attribute__((always_inline)) void heartBeat() {
        static int i = 0;
        i++;
        ledH.write(MyMath::sinDeg(int(i / (!info.isUnderVoltage ? 5 : 1))) / 2 + 0.5);
    }

  private:
    uint16_t photoSensorValueBuff[15];
    Median<uint16_t> medianPhotoValue = Median(photoSensorValueBuff, 15);

    uint8_t batteryValueBuff[15];
    Median<uint8_t> medianBatteryValue = Median(batteryValueBuff, 15);
};

#endif