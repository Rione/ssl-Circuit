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

enum class playType {
    NONE,
    NOTIFY,
    START,
    STOP,
    ERROR,
    SUCCESS,
    WARNING,
    INFO,
};
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
    //   . bit4: doCharge
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
    union {
        struct {
            bool isDetectedBall : 1; // ボール検知（フォトセンサーの検知）2024版のisHolDBall
            bool isHoldBall : 1; // ボール保持（ドリブラーの検知とフォトセンサーの検知の論理積）
            bool isHoldBallReliable : 1; // ボール保持の信頼性 0:信頼性なし 1:信頼性あり、パワーが変わって2秒は信頼性なし                  
        };
        uint8_t data;
    } dribbleStatus; //ボール保持とボール検知の統合

    uint8_t batteryVoltage;
    uint8_t capChargeCertitude; // 0~100

    // local
    volatile uint16_t photoSensorValue;
    bool isUnderVoltage;
    bool isRobotStopFF; // ロボットのFF速度ベクトルも元にロボットが止まっているかを判断する
    bool isKickerChargeMode;
    union {
        struct {
            bool straight : 1;
            bool chip : 1;
        };
        uint8_t status;
    } kickerBoardDoDirectStatus;

} RobotInfo;

typedef struct {
    uint8_t batteryGet;
    union {
        struct {
            bool chargeState : 1;  // stmから送られてくる充電状態
            uint8_t chargeVol : 7; // capChargeCertitude
        };
        uint8_t data;

    } capaData;
    playType buzzer;

} UIRobotInfo_t; // 送るデータ

typedef struct {
    union {
        struct {
            uint8_t mode : 5;
            bool emergencyStop : 1;
            bool charge_state : 1; // 1.切替、0.切替なし
            bool kick : 1;         // キック
        };
        uint8_t data;
    } status;

    uint8_t modePrev = 0;

} UIModeSwitch_t; // main用、一番最初に受け取るデータ

class Robot {
  public:
    Robot();

    // values
    RobotInfo info;

    UIRobotInfo_t UIrobotInfo;
    UIModeSwitch_t UImodeData;

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

    inline __attribute__((always_inline)) void kickStraight(uint8_t power, bool forceSend = false) {
        static Timer timer;
        if (timer.read_ms() < (forceSend ? 10 : 1000)) {
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

    inline __attribute__((always_inline)) void kickChip(uint8_t power, bool forceSend = false) {
        static Timer timer;
        if (timer.read_ms() < (forceSend ? 100 : 1000)) {
            return;
        } else if (timer.read_ms() > 1000) {
            timer.set_ms(1000);
        }
        // doDirectをする時はdata[1]に0xFFを入れてKickerBoardに投げる
        CANBus::CANData canData = {
            .stdId = KICK_CHIP,
            .data = {power, 0, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
        minusCapChargeCertitude(power);
    }

    inline __attribute__((always_inline)) void directStraightKick(uint8_t power) {
        static Timer timer;
        if (timer.read_ms() < 500) {
            return;
        } else if (timer.read_ms() > 500) {
            timer.set_ms(500);
        }
        CANBus::CANData canData = {
            .stdId = KICK_STRAIGHT,
            .data = {power, 0xFF, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
    }

    inline __attribute__((always_inline)) void directChipKick(uint8_t power) {
        static Timer timer;
        if (timer.read_ms() < 500) {
            return;
        } else if (timer.read_ms() > 500) {
            timer.set_ms(500);
        }
        CANBus::CANData canData = {
            .stdId = KICK_CHIP,
            .data = {power, 0xFF, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
    }

    inline __attribute__((always_inline)) void resetDoDirectKick() {
        CANBus::CANData canData = {
            .stdId = KICK_STRAIGHT,
            .data = {0, 0, 0, 0, 0, 0, 0, 0xFF},
        };
        can.send(canData);
    }

    inline __attribute__((always_inline)) void resetDoDirectChipKick() {
        CANBus::CANData canData = {
            .stdId = KICK_CHIP,
            .data = {0, 0, 0, 0, 0, 0, 0, 0xFF},
        };
        can.send(canData);
    }
    inline __attribute__((always_inline)) bool dribble(uint8_t power, bool forceSend = false) {
        static Timer timer;
        static uint8_t dribblePowerPrev = power;
        if (timer.read_ms() > 10000) timer.set_ms(10000);
        if (power == dribblePowerPrev && forceSend == false) {
            if (timer.read_ms() < 100) // パワーが変わっていない場合は送信しない。 forceSendがtrueの場合は100msごとに送信する
                return 0;
        }
        CANBus::CANData canData = {
            .stdId = DRIBBLE_SEND,
            .data = {power, 0, 0, 0, 0, 0, 0, 0},
        };
        can.send(canData);
        timer.reset();
        dribblePowerPrev = power;
        return 1; // 送信した
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
            info.isKickerChargeMode = true;
            led2 = true;
        } else if (value == 0x00) {
            info.isKickerChargeMode = false;
            led2 = false;
        }
    }

    inline __attribute__((always_inline)) void setKickerBoardDoDirectMode(uint8_t value) {

        info.kickerBoardDoDirectStatus.status = value;
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
            resetDoDirectKick();
            resetDoDirectChipKick();
            timer.reset();
        }
    }

    inline __attribute__((always_inline)) void processKicker() {
        // ストレートキックを優先する処理。
        // doDirectXXがtrueの場合はボールセンサが反応した瞬間にキックする
        if (info.kicker.straight > 0) {
            if (info.status.doDirectKick) {
                directStraightKick(info.kicker.straight); // doDirectをKickerBoardに投げる
            } else {
                kickStraight(info.kicker.straight);
            }
        } else if (info.kicker.chip > 0) {
            if (info.status.doDirectChipKick) {
                directChipKick(info.kicker.chip); // doDirectをKickerBoardに投げる
            } else {
                kickChip(info.kicker.chip);
            }
        } else {
            // どっちも0の場合はキックしない
            if (info.status.doDirectKick != info.kickerBoardDoDirectStatus.straight && info.kickerBoardDoDirectStatus.straight) {
                // kickStraight(0, false); // パワー0のキックを投げてdoDirectをリセットする
                resetDoDirectKick();
            }
            if (info.status.doDirectChipKick != info.kickerBoardDoDirectStatus.chip && info.kickerBoardDoDirectStatus.chip) {
                // kickChip(0, false); // パワー0のキックを投げてdoDirectをリセットする
                resetDoDirectChipKick();
            }
        }
    }

    inline __attribute__((always_inline)) void setIsHoldBallValue(uint8_t value) {
        info.dribbleStatus.isHoldBall = (value != 0);
    }

    inline __attribute__((always_inline)) void setIsDetectedBallValue(uint8_t value) {
        info.dribbleStatus.isDetectedBall = (value != 0);
    }

  private:
    uint16_t photoSensorValueBuff[15];
    Median<uint16_t> medianPhotoValue = Median(photoSensorValueBuff, 15);

    uint8_t batteryValueBuff[15];
    Median<uint8_t> medianBatteryValue = Median(batteryValueBuff, 15);

    uint8_t capChargeCertitude; // 0~100
};

#endif