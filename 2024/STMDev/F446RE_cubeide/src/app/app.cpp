#include "app.h"

#include "main.h"
#include "DMAStream.h"
#include "Timer.hpp"
#include "MyMath.hpp"
#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "BufferedSerial.hpp"
#include "CAN.hpp"
#include "BNO055.hpp"

#include "adc.h"

CANBus can(&hcan1, 0x100);
DigitalOut led0(LED0_GPIO_Port, LED0_Pin);
DigitalOut led1(LED1_GPIO_Port, LED1_Pin);
DigitalOut led2(LED2_GPIO_Port, LED2_Pin);
PwmSingleOut ledH(&htim1, TIM_CHANNEL_1);
BufferedSerial serial1(&huart1, 128); // pc
BufferedSerial serial4(&huart4, 128); // xiao
BufferedSerial serial5(&huart5, 256); // RasPi
BNO055 bno(&hi2c1);

typedef struct {
    int16_t motor[4];
    // モータの速度,なぜかMDが2byte受信なのでint16_tにしている
    float driblePower;
    // ドリブルの強さ(速さ),getするデータはuint8_tだが、ハードウェアAPIが　floatになっているのでfloat型
    float kickerPower[2];
    // キッカーの強さ,getするデータはuint8_tだが、ハードウェアAPIが　floatになっているのでfloat型
    volatile uint8_t volt;
    // バッテリー電圧
    uint16_t photoSensor;
    // フォトセンサの1000分率
    bool isHoldBall;
    // ボールを持っているか
    float imuDir;
    float imuDirPrev;
    // PID制御するためにfloat型である必要がある。
    float imuTargetDir;
    // PID制御するためにfloat型である必要がある。
    bool emergency;
    // 危険信号。ロボットに止まって欲しい時にtrueにする
    uint8_t imuStatus;
    // 0:正の角度 1:負の角度 2:IMU0度設定
} RobotInfo;

typedef union {
    int16_t vel; // main value
    struct {
        char L : 8; // LOW byte
        char H : 8; // High byte
    } vel8_t;
} moterOrder;

typedef struct {
    moterOrder M1;
    moterOrder M2;
    moterOrder M3;
    moterOrder M4;
} motorsVel;

CANBus::CANData canRecvData = {
    .stdId = 0x555,
    .data = {100, 200, 0, 0, 0, 0, 0, 8},
};

Timer raspSendTimer;
Timer mdSendTimer;
Timer chargeTimer;

bool isKicked = false;
uint8_t photoSensorValue = 0;
CANBus::CANData canSend_Date;
static bool emergency = false;
motorsVel motors;

inline __attribute__((always_inline)) void heartBeat() {
    static int i = 0;
    i++;
    ledH.write(MyMath::sinDeg(int(i / 5)) / 2 + 0.5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim10) {
        heartBeat(); // 1ms
    } else {
        // pass
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (can.getHcan() == hcan) {
        can.recv(canRecvData);
        led0 = !led0;

        switch (canRecvData.stdId) {
        case 0x123: // フォトセンサの値
            photoSensorValue = canRecvData.data[0];
            break;

        default:
            break;
        }
    }
}

RobotInfo info = {
    .motor = {0, 0, 0, 0},
    .driblePower = 0,
    .kickerPower = {0, 0},
    .volt = 0,
    .photoSensor = 0,
    .isHoldBall = 0,
    .imuDir = 0,
    .emergency = 0,
};

// シリアルで受信したデータを処理するサンプル
uint8_t dataFlame[8] = {0};

void RasRecvSerial() {
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
    const uint8_t HEADER = 0xFF;         // ヘッダ
    const uint8_t dataSize = 10;         // データのサイズ
    static bool headerReceived = false;  // ヘッダを受信したかどうか
    static uint8_t index = 0;            // 受信したデータのインデックスカウンター
    static uint8_t data[dataSize] = {0}; // 受信したデータ

    while (serial5.available()) {
        // 1バイト読み込み
        uint8_t receivedByte = serial5.read();
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
                    info.motor[0] = ((int16_t)data[0] - 100.0) * 5 / 2; //-100 ~ 100 → -250 ~ 250
                    info.motor[1] = ((int16_t)data[1] - 100.0) * 5 / 2;
                    info.motor[2] = ((int16_t)data[2] - 100.0) * 5 / 2;
                    info.motor[3] = ((int16_t)data[3] - 100.0) * 5 / 2;
                    info.driblePower = (float)data[4] / 100;
                    info.kickerPower[0] = (float)data[5] / 10; // ストレート
                    info.kickerPower[1] = (float)data[6] / 10; // チップ
                    info.imuTargetDir = (float)((int16_t)(data[7] * (data[8] % 2 == 1 ? -1 : 1)));

                    info.imuStatus = data[8];
                    info.emergency = data[9];

                    // printf("Ras Recieved:\n");
                    printf("motor[0]: %3d motor[1]: %3d motor[2]: %3d motor[3]: %3d\n", info.motor[0], info.motor[1], info.motor[2], info.motor[3]);

                    headerReceived = false; // 次のヘッダを待つ準備をする
                    index = 0;              // インデックスをリセット
                }
            }
        }
    }
}

void RasSendSerial(RobotInfo &info) {
    const uint8_t dataSize = 6;     // データのサイズ
    uint8_t buffer[dataSize] = {0}; // データ
    uint8_t startBytes[4] = {0xFF, 0, 0xFF, 0};

    buffer[0] = info.volt;                 //
    buffer[1] = info.photoSensor >> 8;     //
    buffer[2] = info.photoSensor & 0x00FF; // LSB
    buffer[3] = info.isHoldBall;
    buffer[4] = (int16_t)(info.imuDir) >> 8;     // MSB
    buffer[5] = (int16_t)(info.imuDir) & 0x00FF; // LSB

    serial5.write(startBytes, 4);
    serial5.write(buffer, dataSize);

    // printf("Ras Send:\n");
}

uint32_t readADC1() {
    // ADCの値を読み取る
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    return HAL_ADC_GetValue(&hadc1);
}

void sendMotorValues(motorsVel *motors) {
    if (emergency == true) {
        motors->M1.vel = 0;
        motors->M2.vel = 0;
        motors->M3.vel = 0;
        motors->M4.vel = 0;
    }
    canSend_Date.stdId = 0x1AA;
    canSend_Date.data[0] = motors->M1.vel8_t.L;
    canSend_Date.data[1] = motors->M1.vel8_t.H;
    canSend_Date.data[2] = motors->M2.vel8_t.L;
    canSend_Date.data[3] = motors->M2.vel8_t.H;
    canSend_Date.data[4] = motors->M3.vel8_t.L;
    canSend_Date.data[5] = motors->M3.vel8_t.H;
    canSend_Date.data[6] = motors->M4.vel8_t.L;
    canSend_Date.data[7] = motors->M4.vel8_t.H;

    can.send(canSend_Date);
}

void setVelocityZero() {
    motors.M1.vel = 0;
    motors.M2.vel = 0;
    motors.M3.vel = 0;
    motors.M4.vel = 0;
    sendMotorValues(&motors);
}

void setVelocity(RobotInfo &info, int8_t turn) {

    motors.M1.vel = info.motor[0] + turn;
    motors.M2.vel = info.motor[1] + turn;
    motors.M3.vel = info.motor[2] + turn;
    motors.M4.vel = info.motor[3] + turn;

    // motors.M1.vel = 30;
    // motors.M2.vel = 30;
    // motors.M3.vel = 30;
    // motors.M4.vel = 30;

    // printf("Motor[0]: %3d Motor[1]: %3d Motor[2]: %3d Motor[3]: %3d\n", motors.M1.vel, motors.M2.vel, motors.M3.vel, motors.M4.vel);
    sendMotorValues(&motors);
}

// SubBoard functions

uint8_t getCAN_TEC() {
    // CAN エラーステータスレジスタ(CAN_ESR)
    // 250回エラーが起きると止まる
    return ((hcan1.Instance->ESR) >> 16) & 0xFF;
}

void sendBoosterChargeStart() {
    CANBus::CANData canSubBoardData = {
        .stdId = 0x10,
        .data = {0},
    };
    can.send(canSubBoardData);
}

void sendKickPowerStraight(float power) {
    CANBus::CANData canSubBoardData = {
        .stdId = 0x12,
        .data = {0},
    };
    can.send(canSubBoardData);
    isKicked = true;
}

void sendKickPowerChip(float power) {
    CANBus::CANData canSubBoardData = {
        .stdId = 0x11,
        .data = {0},
    };
    can.send(canSubBoardData);
    isKicked = true;
}

void sendDribblePower(float power) {
    CANBus::CANData canSubBoardData;
    if (power > 0.5) {
        canSubBoardData = {
            .stdId = 0x13,
            .data = {0},
        };
    } else {
        canSubBoardData = {
            .stdId = 0x14,
            .data = {0},
        };
    }
    can.send(canSubBoardData);
}

void setup() {
    for (size_t i = 0; i < 10; i++) {
        led1 = bno.check();
        HAL_Delay(50);
    }
    bno.setUnit(1, 1, 1, 0);
    bno.setPowerMode();
    // bno.setOperaitonMode(OPERATION_MODE_AMG);
    bno.setOperaitonMode(OPERATION_MODE_NDOF);
    // bno.accConfig();
    bno.init();
    can.init();

    serial1.init();
    serial4.init();
    serial5.init();
    ledH.init();
    printf("Hello World\n");

    setVelocityZero();
    raspSendTimer.reset();
    mdSendTimer.reset();
    chargeTimer.reset();

    bno.setAttitudeZero();

    wait_ms(1000);
    sendBoosterChargeStart();
}

void main_app() {
    setup();
    led0 = 0;
    emergency = false;
    double voltage = 0;
    bool imuDirEnable = true;
    while (1) {
        info.photoSensor = photoSensorValue;
        if (photoSensorValue < 250) {
            info.isHoldBall = true;
        } else {
            info.isHoldBall = false;
        }
        if (raspSendTimer.read_ms() > 15.0) {
            RasSendSerial(info);
            raspSendTimer.reset();
        }

        RasRecvSerial();

        // IMUの状態が2の時、IMUをリセットする
        if (info.imuStatus == 2) {
            bno.setAttitudeZero();
        }
        if (info.imuStatus == 9) {
            imuDirEnable = false;
        } else {
            imuDirEnable = true;
        }

        if (info.imuStatus == 2 || info.imuStatus == 3) {
            bno.setAngle(info.imuTargetDir);
        }

        float angle = bno.getAttitude();
        int16_t turn = angle * 80;
        if (turn > 100) turn = 80;
        if (turn < -100) turn = -80;
        printf("angle:%f turn:%d\n", angle, (int8_t)turn);

        if (mdSendTimer.read_ms() > 2.0) {
            setVelocity(info, turn);
            mdSendTimer.reset();
            // print CAN TEX(23bit to 16bit)を取り出して表示
            // printf("TEC:%d\n", getCAN_TEC());
            // print REC as bit
            // if (getCAN_TEC() > 230) {
            //     resetCAN_TEC_REC();
            // }
            // printf("TECb:\n");
            // for (int i = 0; i < 32; i++) {
            //     printf("%d", (hcan1.Instance->ESR >> i) & 1);
            // }
            // printf("\n");
        }

        // Kicker
        if (info.kickerPower[0] > 0 || info.kickerPower[1] > 0) {
            // キック。ストレートを優先したい
            if (info.kickerPower[0] > 0) { // ストレート
                sendKickPowerStraight(info.kickerPower[0]);
            } else if (info.kickerPower[1] > 0) { // チップ
                sendKickPowerChip(info.kickerPower[1]);
            }
            chargeTimer.reset();
            isKicked = true;
        }
        // 充電
        if (isKicked && chargeTimer.read_ms() > 500) {
            sendBoosterChargeStart();
            isKicked = false;
        }

        // ドリブル
        sendDribblePower(info.driblePower);
    }
}