#include "RobotSequence.hpp"

// Robot.hppとの切り分け
// class Robot内の関数を使用したシーケンスの記述

extern Robot robot;

void stopRobot(uint16_t interval) {
    // ロボットの動作を停止
    static Timer timer;
    if (timer.read_ms() > interval) {
        robot.sendDribble(0);
        robot.motorDriver.sendEmg();
        robot.kickerBoard.resetDoDirect(STRAIGHT);
        robot.kickerBoard.resetDoDirect(CHIP);
        timer.reset();
    }
}

void uiKickControl(RobotInfo_t &info) {
    // UIからの充電制御
    if (info.uiStatus.chargeStateChange == 1) {
        // 充放電の切り替え
        if(info.isKickerChargeMode == false) {
            robot.kickerBoard.chargeControl(CHARGE);
            printf("UI Start charge\n");
            robot.led2 = true;
        } else {
            robot.kickerBoard.chargeControl(DISCHARGE);
            printf("UI Start discharge\n");
            robot.led2 = false;
        }
        info.uiStatus.chargeStateChange = 0;
        robot.manageByUserCounter.reset();
    } else if(info.uiStatus.kick == 1) {
        // キック
        robot.kickerBoard.kick(STRAIGHT, 50);
        printf("kick\n");
        info.uiStatus.kick = 0;
        robot.manageByUserCounter.reset();
    }   
}

void buzzerControl(RobotInfo_t &info) {
    // BUZZERの制御
    // doDirectKick, doDirectChipKickの状態に応じてBUZZERを鳴らす
    if (info.status.doDirectKick || info.status.doDirectChipKick) {
        info.buzzer = playType::SUCCESS;
    } else {
        info.buzzer = playType::NONE;
    }
}

void swKickControl(RobotInfo_t &info) {
    // スイッチの状態でキックを制御
    static Timer doChargeTimer;
    if (robot.manageByUserCounter.read_ms() > 15000) robot.manageByUserCounter.set_ms(15000); // オーバーフローを防ぐ

    if (robot.swDischarge.isRelease()) {
        if (robot.swDischarge.readPressedTime() > 1600) {
            robot.kickerBoard.chargeControl(DISCHARGE);
            robot.led2 = false;
            // printf("discharge start\n");
        } else if (robot.swDischarge.readPressedTime() > 800) {
            robot.kickerBoard.chargeControl(CHARGE);
            robot.led2 = true;
            // printf("charge start\n");
        } else if (robot.swDischarge.readPressedTime() > 200) {
            robot.kickerBoard.kick(STRAIGHT, 50);
            // printf("kick\n");
        }
        robot.manageByUserCounter.reset();

    } else {
        if (robot.manageByUserCounter.read_ms() >= 15000) { // ユーザーがスイッチでキッカーの充電or放電をしてから15秒以上経過したらPiの指示に従う
            if (doChargeTimer.read_ms() > 100) {      // 100msごとに実行
                if (robot.info.status.doCharge == true) {
                    static uint8_t countD = 0;
                    // Piから充電しろと言われている。
                    if (robot.info.isKickerChargeMode == false) {
                        // KickerBoardから充電していないとの情報を得ている。噛み合っていない
                        countD++;
                        if (countD > 10) {
                            robot.kickerBoard.chargeControl(CHARGE);
                            // printf("charge from Pi\n");
                            countD = 0;
                        }
                    }
                } else {
                    // Piから放電しろと来ている
                    static uint8_t countC = 0;
                    if (robot.info.isKickerChargeMode == true) {
                        countC++;
                        if (countC > 10) {
                            robot.kickerBoard.chargeControl(DISCHARGE);
                            // printf("discharge from Pi\n");
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


void ballMoving() {
    // ボール保持したまま移動する
    uint8_t state = 0;
    Timer stateSwitchTimer;
    uint16_t velX = 0;
    uint16_t time;
    while (1) {
        time = stateSwitchTimer.read_ms();

        switch (state) {
        case 0:
            velX = 1500 * MyMath::sinDeg(time * 0.18 * 0.5); // 加速のためにタイマーを使った sinの加速をしてる
            robot.motorDriver.setVelocityFF(velX, 0, 0);     // 前進
            robot.sendDribble(100);                       // ドリブラー100%
            if (time > 2000) {
                state = 1;
                stateSwitchTimer.reset(); // 2秒経ったら状態遷移
            }
            printf("0\n");
            break;
        case 1:
            robot.motorDriver.setVelocityFF(600, -600, 3600); // 斜め前に進みながら回転(CMDragonsのTDPより)
            robot.sendDribble(100);
            if (time > 500) {
                state = 0;
                stateSwitchTimer.reset();
            }
            printf("1\n");
            break;
        }
        HAL_Delay(20);
    }
}

void dribbleHoldBack() {
    // ボールを保持しながら後退する
    Timer timer;
    Timer initialHoldtimer;
    static bool ballHoldPrev = false;
    timer.reset();

    static int velX = 0;
    while (1){
        robot.sendDribble(100);
        if(robot.info.dribbleStatus.isHoldBall == true) {
            if (ballHoldPrev != robot.info.dribbleStatus.isHoldBall) {
                initialHoldtimer.reset();// first time hold ball
            }
            if (initialHoldtimer.read_ms() > 300){ // 開始300ms以降から加速
                if (timer.read_ms() > 10 && velX > -700) {
                    velX -= 5;
                    timer.reset();
                }
            }
            robot.motorDriver.setVelocityFF(velX, 0, 0);
        } else {
            velX = 0;
            robot.motorDriver.setVelocityFF(0, 0, 0);
        }
        printf("velX: %d\n", velX);
        HAL_Delay(10);
        ballHoldPrev = robot.info.dribbleStatus.isHoldBall;
    }
}
