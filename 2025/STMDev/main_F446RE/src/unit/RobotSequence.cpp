#include "RobotSequence.hpp"

// Robot.hppとの切り分け
// class Robot内の関数を使用したシーケンスの記述

extern Robot robot;

void stopRobot(uint16_t interval) {
    // ロボットの動作を停止
    static Timer timer;
    if (timer.read_ms() > interval) {
        robot.dribble(0);
        robot.motorDriver.sendEmg();
        robot.kickerBoard.resetDoDirect(STRAIGHT);
        robot.kickerBoard.resetDoDirect(CHIP);
        timer.reset();
    }
}

void sendKicker(RobotInfo_t &info) {
    // キックの処理
    // ストレートを優先してキック
    if (info.kicker.straight > 0) {
        robot.kickerBoard.kick(STRAIGHT, info.kicker.straight, info.status.doDirectKick);
    } else if (info.kicker.chip > 0) {
        robot.kickerBoard.kick(CHIP, info.kicker.chip, info.status.doDirectChipKick);
    } else {
        // どっちも0の場合はキックしない
        if (info.status.doDirectKick != info.kickerBoardDoDirectStatus.straight && info.kickerBoardDoDirectStatus.straight) {
            // kickStraight(0, false); // パワー0のキックを投げてdoDirectをリセットする
            robot.kickerBoard.resetDoDirect(STRAIGHT);
        }
        if (info.status.doDirectChipKick != info.kickerBoardDoDirectStatus.chip && info.kickerBoardDoDirectStatus.chip) {
            // kickChip(0, false); // パワー0のキックを投げてdoDirectをリセットする
            robot.kickerBoard.resetDoDirect(CHIP);
        }
    }
}

void sendMotor(RobotInfo_t &info, uint8_t interval) {
    // MDの処理
    // モータードライバの送信速度決め
    int16_t __velX = robot.meanVelX.calc((float)info.velX.vel);
    int16_t __velY = robot.meanVelY.calc((float)info.velY.vel);
    int16_t __velAngler = robot.meanVelAngler.calc((float)info.velAngler.vel);
    static uint8_t md_sendcount = 0;
    md_sendcount ++;
    if (md_sendcount % interval == 0) { // 5msごとに送信
        robot.motorDriver.setVelocityFF(__velX, __velY, __velAngler);
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