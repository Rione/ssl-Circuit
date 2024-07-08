#include "Mode.hpp"
#include "Average.h"
#include "Median.h"
class MainMode : public Mode {
  public:
    MainMode(char letter, const char name[], Robot *robotPtr) : Mode(letter, name, robotPtr) {}
    void before() override;
    void after() override;
    void loop() override;

  private:
    float meanVelXBuf[15];
    float meanVelYBuf[15];
    float meanVelAngBuf[15];

    Average<float> meanVelX = Average(meanVelXBuf, 15);
    Average<float> meanVelY = Average(meanVelYBuf, 15);
    Average<float> meanVelAngler = Average(meanVelAngBuf, 15);

    Timer timer;

    inline void boosterManager() {
        static Timer uiBoosterCheckInterval;
        static Timer doChargeTimer;
        static Timer manageByUserCounter;
        // ロボットの状態に関わらず常に行う処理
        if (manageByUserCounter.read_ms() > 15000) manageByUserCounter.set_ms(15000); // オーバーフローを防ぐ
        if (uiBoosterCheckInterval.read_ms() > 1000) uiBoosterCheckInterval.set_ms(1000);

        // UIとの通信
        if(uiBoosterCheckInterval.read_ms() > 100) {
            uiBoosterCheckInterval.reset();
            
            //　パケットに代入
            robot->UIrobotInfo.batteryGet = robot->info.batteryVoltage;
            robot->UIrobotInfo.capaData.chargeState = robot->info.isKickerChargeMode;
            robot->UIrobotInfo.capaData.chargeVol = robot->info.capChargeCertitude;

            // send the state to ui
            robot->serial4.write(0xFF);
            robot->serial4.write(robot->UIrobotInfo.batteryGet);
            robot->serial4.write(robot->UIrobotInfo.capaData.data);

            // check 
            if(robot->serial4.available()){
                static const uint8_t HEADER = 0xFF;  // ヘッダ
                static const uint8_t dataSize = 1;  // データのサイズ
                static bool headerReceived = false;  // ヘッダを受信したかどうか
                static uint8_t index = 0;            // 受信したデータのインデックスカウンター
                static uint8_t data[dataSize] = {0}; // 受信したデータ

                printf("get");
                
                while(robot->serial4.available()){
                    uint8_t receivedByte = robot->serial4.read();
                    if(!headerReceived){
                        index = 0;
                        if(receivedByte == HEADER) headerReceived = true;
                    }else{
                        data[index] = receivedByte;
                        index++;
                        if(index == dataSize){
                            robot->UImodeData.status.data = data[0];
                            headerReceived = false;
                            index = 0;
                        }
                    }
                }

                if(robot->UImodeData.status.charge_state == 1){
                    if(robot->info.isKickerChargeMode == false){
                        robot->chargeStart();
                        // printf("UI Start charge\n");
                        robot->led2 = true;
                        // robot->serial4.write(0x01);
                    } else {
                        robot->discharge();
                        // printf("UI Start discharge\n");
                        robot->led2 = false;
                        // robot->serial4.write(0x00);
                    }
                    
                    robot->UImodeData.status.charge_state = 0;

                }else if(robot->UImodeData.status.kick == 1){
                    robot->kickStraight(50);    
                    printf("kick\n");
                    robot->UImodeData.status.kick = 0;
                }

                manageByUserCounter.reset();
            }
        }


        if (robot->swDischarge.isRelease()) {
            if (robot->swDischarge.readPressedTime() > 1600) {
                robot->discharge();
                robot->led2 = false;
                printf("discharge start\n");
            } else if (robot->swDischarge.readPressedTime() > 800) {
                robot->chargeStart();
                robot->led2 = true;
                printf("charge start\n");
            } else if (robot->swDischarge.readPressedTime() > 200) {
                robot->kickStraight(100);
                printf("kick\n");
            }
            manageByUserCounter.reset();
        } else {
            if (manageByUserCounter.read_ms() >= 15000) { // ユーザーがスイッチでキッカーの充電or放電をしてから15秒以上経過したらPiの指示に従う
                if (doChargeTimer.read_ms() > 100) {      // 100msごとに実行
                    if (robot->info.status.doCharge == true) {
                        static uint8_t countD = 0;
                        // Piから充電しろと言われている。
                        if (robot->info.isKickerChargeMode == false) {
                            // KickerBoardから充電していないとの情報を得ている。噛み合っていない
                            countD++;
                            if (countD > 10) {
                                robot->chargeStart();
                                printf("charge from Pi\n");
                                countD = 0;
                            }
                        }
                    } else {
                        // Piから放電しろと来ている
                        static uint8_t countC = 0;
                        if (robot->info.isKickerChargeMode == true) {
                            countC++;
                            if (countC > 10) {
                                robot->discharge();
                                printf("discharge from Pi\n");
                                countC = 0;
                            }
                            // KickerBoardから充電しているとの情報を得ている。噛み合っていない
                        }
                    }
                    doChargeTimer.reset();
                }
            }
        }
    }
};
