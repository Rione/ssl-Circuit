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
    //   . bit5: isSignalReceived
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
            bool doDirectKick : 1;     // ボールセンサが反応した瞬間にストレートキックする
            bool doDirectChipKick : 1; // ボールセンサが反応した瞬間にチップキックする
            bool reserved0 : 1;
            bool doCharge : 1;         // 0: discharge, 1: doCharge
            bool isSignalReceived : 1; // コントローラから信号か出ているときにtrue
            bool isCtrlByRobot : 1;    // (0: RACOON-Ctrl, 1: Robot-local-Ctrl) 位置制御をローカルで行うか否か
            bool parity : 1;           // パリティビット 今の所使ってない。
        };
        uint8_t data;
    } status;

    // Infomation STM32→RaspberryPi
    uint8_t isHoldBall;
    uint8_t batteryVoltage;
    uint8_t capChargeCertitude; // 0~100

    // local
    volatile uint16_t photoSensorValue;
    bool isUnderVoltage;
    bool isRobotStopFF; // ロボットのFF速度ベクトルも元にロボットが止まっているかを判断する

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
    Button swImu = Button(IMU_SW_GPIO_Port, IMU_SW_Pin);
    Button swDischarge = Button(DISCHARGE_SW_GPIO_Port, DISCHARGE_SW_Pin);

    BufferedSerial serial1 = BufferedSerial(&huart1, 128); // pc
    BufferedSerial serial4 = BufferedSerial(&huart4, 128); // xiao
    BufferedSerial serial5 = BufferedSerial(&huart5, 256); // RasPi

    MotorDriver motorDriver = MotorDriver(&can);

    Timer rasSendInterval = Timer();

    void hardwareInit();
    void rasRecvSerial();
    void rasSendSerial(RobotInfo &info, uint16_t interval);
    void getSensors(RobotInfo *info);

    inline __attribute__((always_inline)) void heartBeat() {
        static int i = 0;
        i++;
        ledH.write(MyMath::sinDeg(int(i / (!info.isUnderVoltage ? 1 : 5))) / 2 + 0.5);
    }

    inline __attribute__((always_inline)) void chargeStart() {
        CANBus::CANData canData = {
            .stdId = CHARGE_START,
            .data = {0},
        };
        can.send(canData);
    }

    inline __attribute__((always_inline)) void discharge() {
        CANBus::CANData canData = {
            .stdId = DISCHARGE_START,
            .data = {0},
        };
        can.send(canData);
        minusCapChargeCertitude(100);
    }

    inline __attribute__((always_inline)) void kickStraight(uint8_t power) {
        static Timer timer;
        if (timer.read_ms() < 1000) {
            return;
        } else if (timer.read_ms() > 1000) {
            timer.set_ms(1000);
        }
        CANBus::CANData canData = {
            .stdId = KICK_STRAIGHT,
            .data = {power, 0, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
        minusCapChargeCertitude(power);
    }

    inline __attribute__((always_inline)) void kickChip(uint8_t power) {
        static Timer timer;
        if (timer.read_ms() < 1000) {
            return;
        } else if (timer.read_ms() > 1000) {
            timer.set_ms(1000);
        }
        CANBus::CANData canData = {
            .stdId = KICK_CHIP,
            .data = {power, 0, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
        minusCapChargeCertitude(power);
    }

    inline __attribute__((always_inline)) void dribble(uint8_t power, bool forceSend = false) {
        static Timer timer;
        static uint8_t dribblePowerPrev = power;
        if (power == dribblePowerPrev && forceSend == false) {
            if (timer.read_ms() < 100) // パワーが変わっていない場合は送信しない。 forceSendがtrueの場合は100msごとに送信する
                return;
        }
        CANBus::CANData canData = {
            .stdId = DRIBBLE,
            .data = {power, 0, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
        dribblePowerPrev = power;
    }

    inline __attribute__((always_inline)) void setPhotoSensorValue(uint16_t value) {
        info.photoSensorValue = value;
    }

    inline __attribute__((always_inline)) void setChageDoneSignal(uint16_t value) {
        if (value == 0xFF) { // Done
            capChargeCertitude = 100;
            led2 = true;
        }
    }

    inline __attribute__((always_inline)) void setKickerBoardChargeMode(uint16_t value) {
        if (value == 0xFF) { // Done
            isKickerChargeMode = true;
            led2 = true;
        } else if (value == 0x00) {
            isKickerChargeMode = false;
            led2 = false;
        }
    }

    inline __attribute__((always_inline)) void minusCapChargeCertitude(uint8_t value) {
        if (capChargeCertitude < value) {
            capChargeCertitude = 0;
            return;
        }
        capChargeCertitude -= value;
    }

    inline __attribute__((always_inline)) uint8_t getCapChargeCertitude() {
        return capChargeCertitude;
    }

    // FF速度ベクトルからロボットが止まっているかを判断する
    inline __attribute__((always_inline)) void checkRobotRest() {
        static int velZeroCount = 0;
        if (info.velX.vel == 0 && info.velY.vel == 0 && info.velAngler.vel == 0) {
            if (velZeroCount < 1000) {
                velZeroCount++;
            } else {
                velZeroCount = 1000;
                info.isRobotStopFF = true;
            }
        } else {
            velZeroCount = 0;
            info.isRobotStopFF = false;
        }
    }

    inline __attribute__((always_inline)) void stopRobot(uint16_t interval) {
        static Timer timer;
        if (timer.read_ms() > interval) {
            dribble(0);
            motorDriver.sendEmg();
            timer.reset();
        }
    }

    inline __attribute__((always_inline)) void processKicker() {
        // ストレートキックを優先する処理。
        // doDirectXXがtrueの場合はボールセンサが反応した瞬間にキックする
        if (info.kicker.straight > 0) {
            if (info.status.doDirectKick) {
                if (info.isHoldBall)
                    kickStraight(info.kicker.straight);
            } else {
                kickStraight(info.kicker.straight);
            }
        } else if (info.kicker.chip > 0) {
            if (info.status.doDirectChipKick) {
                if (info.isHoldBall)
                    kickChip(info.kicker.chip);
            } else {
                kickChip(info.kicker.chip);
            }
        }
    }

  private:
    uint16_t photoSensorValueBuff[15];
    Median<uint16_t> medianPhotoValue = Median(photoSensorValueBuff, 15);

    uint8_t batteryValueBuff[15];
    Median<uint8_t> medianBatteryValue = Median(batteryValueBuff, 15);

    Timer lastChargeDoneTime;
    uint8_t capChargeCertitude; // 0~100
    bool isKickerChargeMode;
};

#endif