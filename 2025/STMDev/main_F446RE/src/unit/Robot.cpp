#include "Robot.hpp"

#include <math.h>

#include "RasProtocol.hpp"

// 電源を切らないリセット(ウォッチドッグ)の前後で残すジャイロバイアス。高速起動時はキャリブレーションの代わりに使う
// (試合中は機体が動いている可能性があり、その場でキャリブレーションすると誤ったバイアスになる)
static const uint32_t IMU_PERSIST_MAGIC = 0x494D5542;  // "IMUB"
static struct {
      uint32_t magic;
      gyro_t gyroBias;
} imuPersist __attribute__((section(".noinit")));

Robot::Robot() {
}

void Robot::hardwareInit(bool fastBoot) {
      bno.recoverBus();  // 前回I2Cの途中でリセットされていてもバスを使える状態にする
      // IMUを載せていない機体もあるので、起動時に一度だけ有無を確かめ、無ければIMUの処理を全て省く
      // (省かないと、キャリブレーションで待たされたり、I2C線が浮いている場合は読み出しのたびに待たされる)。
      // BNO055は電源投入から応答するまで約650msかかるので、通常起動では起動後1秒までは確かめ直す。
      // ウォッチドッグによる高速起動ではBNO055の電源は入ったままなので1回だけ確かめる
      imuAvailable = bno.check();
      while (!imuAvailable && !fastBoot && HAL_GetTick() < IMU_DETECT_TIMEOUT_MS) {
            HAL_Delay(50);
            // 前の起動のI2Cの途中で止まった等で、BNO055がバスを保ったままのことがある。確かめ直す前に毎回復旧する
            bno.recoverBus();
            imuAvailable = bno.check();
      }
      led1 = imuAvailable;
      if (imuAvailable) {
            // 書き込みがI2Cの不調で抜けると、BNO055はCONFIGモードのまま残り、データのレジスタが全て0になる
            // (読み出し自体は成功するので、0がそのまま正しい値として使われていた)。読み返して確かめ、合わなければ設定し直す
            bool configured = false;
            for (uint8_t i = 0; i < IMU_CONFIG_RETRY && !configured; i++) {
                  if (i > 0) {
                        imuReconfigCount++;
                        bno.recoverBus();
                  }
                  configured = bnoConfigure();
            }
            if (!configured) {
                  printf("BNO055: operation mode did not take effect (will retry while running)\n");
            }
            // bno.accConfig();
            bno.init();
            // bno.getCalibration();
      }

      can.init();

      serial1.init();
      serial4.init();
      serial5.init();

      ledH.init();
      printf("Hello World\n");

      medianPhotoValue.init();
      medianBatteryValue.init();

      filter.begin(33000);  // 4000のはずなんだけど、何故か8.25倍しないとmain_appで使い物にならなかった...

      // mpu.init();

      // bno.setAttitudeZero();
      if (!imuAvailable) {
            printf("IMU not available: skip IMU calibration and reading\n");
      }
#if IMU_FUSION == IMU_FUSION_STM
      if (imuAvailable && fastBoot && imuPersist.magic == IMU_PERSIST_MAGIC) {
            bnoGyroBias = imuPersist.gyroBias;
            bnoFilter.setBeta(IMU_MADGWICK_BETA);
            bnoDtTimer.reset();
            printf("IMU fast boot: reuse GyroBias[dps] X:%+.3f Y:%+.3f Z:%+.3f\n", bnoGyroBias.x, bnoGyroBias.y, bnoGyroBias.z);
            return;
      }
#endif
      if (!fastBoot) {
            HAL_Delay(1000);
      }

      //   bnoCalibrate();
#if IMU_FUSION == IMU_FUSION_STM
      if (imuAvailable) {
            bnoGyroCalibrate();
      }
#endif
}

// Piからの指令を受け取る。パケット = COBS(ペイロード20byte ‖ CRC16) ‖ 0x00 (docs/UART_PROTOCOL.md)。
// 0x00が来たらそこがパケットの区切り。CRC・長さ・typeが合うものだけを使い、合わないものは捨てて数える
// (旧形式はチェックサムが無く、線が浮いているとノイズが正しいパケットとして解釈されていた)。
// 旧形式(0xFFで始まる19byte)の指令は形が合わないので使わない (RACOON-Pi2では動かせない)。
void Robot::rasRecvSerial(RobotInfo_t &info) {
      static uint8_t buf[RAS_RX_PACKET_MAX];  // 区切りまでのバイト (COBSで符号化されたまま)
      static uint8_t len = 0;
      static bool skipping = false;           // 区切りが来ないまま長くなったので、次の0x00まで読み捨てている
      static Timer timeoutTimer;              // 最後にバイトを受信してからの経過時間
      // 最後に正しいパケットを受信した時刻[ms]。DWTのTimerは約23.8秒で一周するので、長時間の途絶も
      // 判定できるようHAL_GetTickを使う
      static uint32_t lastPacketTick = 0;
      // パケットの最初のバイトを受け取った時点の受信エラー数。パケットの途中で受信エラー(ノイズ等)が
      // 起きていたら、CRCが合っていてもそのパケットは捨てる
      static uint32_t rxErrorCountAtStart = 0;

      // Piが落ちた・ケーブルが抜けた場合、最後の指令(速度・キック・isSignalReceived)が残ったままだと
      // ロボットが最後の速度で走り続けてしまう。一定時間パケットが来なければ指令を捨てて停止させる
      if (HAL_GetTick() - lastPacketTick > RAS_SIGNAL_TIMEOUT_MS && info.status.data != 0) {
            info.status.data = 0;  // isSignalReceived=0 → 各モードで待機(停止)処理になる
            info.velX.vel = 0;
            info.velY.vel = 0;
            info.velAngler.vel = 0;
            info.dribblePower = 0;
            info.kicker.straight = 0;
            info.kicker.chip = 0;
            rasSignalTimeoutCount++;
      }

      if (!serial5.available() && timeoutTimer.read_ms() > 1000) {
            // 1秒間受信がない場合は受信状態とバッファをリセットする
            serial5.flush();
            len = 0;
            skipping = false;
            timeoutTimer.reset();
            return;
      }

      while (serial5.available()) {
            const uint8_t receivedByte = serial5.read();
            timeoutTimer.reset();

            if (receivedByte != ras::kDelimiter) {
                  if (skipping) {
                        continue;
                  }
                  if (len == 0) {
                        rxErrorCountAtStart = serial5.rxErrorCount;
                  }
                  if (len >= sizeof(buf)) {
                        // 区切りが来ないまま長くなった (ノイズ・ボーレートの誤り)。次の0x00まで読み捨てる
                        rasCrcErrorCount++;
                        skipping = true;
                        len = 0;
                        continue;
                  }
                  buf[len++] = receivedByte;
                  continue;
            }

            // 区切りの0x00
            const uint8_t n = len;
            len = 0;
            if (skipping) {
                  skipping = false;
                  continue;
            }
            if (n == 0) {
                  continue;  // 区切りが続いた
            }
            if (serial5.rxErrorCount != rxErrorCountAtStart) {
                  // パケットの途中で受信エラーが起きていたので、中身を信用せず捨てる
                  rasPacketErrorCount++;
                  continue;
            }
            uint8_t payload[ras::kCommandPayloadSize];
            if (ras::openPacket(buf, n, ras::kTypeCommand, ras::kCommandPayloadSize, payload) != ras::OpenResult::kOk) {
                  rasCrcErrorCount++;  // CRC・長さ・typeが合わない
                  continue;
            }

            // 18byteの並びは旧形式(0xFFのあとの18byte)と同じ
            const uint8_t *data = &payload[ras::kCommandBody];
            info.velX.L = data[0];
            info.velX.H = data[1];
            info.velY.L = data[2];
            info.velY.H = data[3];
            info.velAngler.L = data[4];
            info.velAngler.H = data[5];
            info.dribblePower = data[6];
            info.kicker.straight = data[7];
            info.kicker.chip = data[8];

            info.relativePositionX.L = data[9];
            info.relativePositionX.H = data[10];
            info.relativePositionY.L = data[11];
            info.relativePositionY.H = data[12];
            info.relativeTheta.L = data[13];
            info.relativeTheta.H = data[14];
            info.camera.x = data[15];
            info.camera.y = data[16];

            info.status.data = data[17];
            lastPacketTick = HAL_GetTick();
      }
}

// Piへテレメトリーを送る。パケット = COBS(ペイロード22byte ‖ CRC16) ‖ 0x00 (docs/UART_PROTOCOL.md)。
// IMUの値は、IMUが見つかっていて直近RAS_IMU_STALE_MS以内に新しい値が読めたときだけ載せ、
// statusのbit3 (imu_valid) を立てる。単位とスケールは新機体のSPIと同じ。
void Robot::rasSendSerial(RobotInfo_t &info, uint16_t interval) {
      static const float GRAVITY = 9.80665f;          // [m/s^2] BNO055の加速度 [m/s^2] → [g]
      static const float DEG2RAD = 0.01745329252f;    // BNO055のジャイロ [dps] → [rad/s]
      static Timer timer;

      if (timer.read_ms() < interval) {
            return;
      }

      info.dribbleStatus.isNewDrib = newDribblerDetected;  // 2025年版ドリブラー基板か

      ras::Telemetry t = {};
      t.seq = rasTxSeq++;
      t.batteryDeciVolts = info.batteryVoltage;
      t.capValEstimate = info.capValEstimate;
      // ホイールはモーターの番号の順 (M0 55°、M1 135°、M2 -135°、M3 -55°)。Wingのwheel0〜3と同じ
      const int16_t *motor = info.mdStatus.motorAngularVelocity;
      for (int i = 0; i < 4; i++) {
            t.wheel[i] = MotorDriver::motorToWheelScaled(motor[i]);
      }

      const bool imuValid = imuAvailable && (HAL_GetTick() - imuLastUpdateTick) <= RAS_IMU_STALE_MS;
      t.status = (uint8_t)((info.dribbleStatus.data & ras::kStatusSensorMask) | (imuValid ? ras::kStatusImuValid : 0));
      if (imuValid) {
            t.accelX = ras::scaleToInt16(info.imuState.acc.x / GRAVITY, 1000.0f);
            t.accelY = ras::scaleToInt16(info.imuState.acc.y / GRAVITY, 1000.0f);
            t.yawRate = ras::scaleToInt16(info.imuState.gyro.z * DEG2RAD, 900.0f);
#if IMU_FUSION == IMU_FUSION_BNO_IMUPLUS
            // BNO055のフュージョンのyawは時計回りが正なので、反時計回りが正になるよう反転する
            t.yaw = ras::scaleToInt16(-info.imuState.euler.yaw, 10000.0f);
#else
            t.yaw = ras::scaleToInt16(info.imuState.euler.yaw, 10000.0f);
#endif
      }
      t.rxBad = (uint8_t)(rasPacketErrorCount + rasCrcErrorCount);  // 255の次は0

      uint8_t payload[ras::kTelemetryPayloadSize];
      ras::packTelemetry(t, payload);
      uint8_t packet[ras::kTelemetryPacketSize];
      const size_t n = ras::buildPacket(payload, sizeof(payload), packet);
      serial5.write(packet, (uint16_t)n);  // 26byteなのでDMAの送信バッファ(64byte)に収まる
      timer.reset();
}

void Robot::getSensors(RobotInfo_t *info) {
      static uint16_t underVoltageCount = 0;
      // バッテリー電圧
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 100);
      uint32_t batt = HAL_ADC_GetValue(&hadc1);
      info->batteryVoltage = medianBatteryValue.calc((uint8_t)((float)(batt) * 3.3f / 4095.0f * 58.5f - 5));
      if (info->batteryVoltage < BATTERY_CUT_OFF) {
            if (underVoltageCount < 1000) underVoltageCount++;
      } else {
            if (underVoltageCount > 0) underVoltageCount--;
      }
      info->isUnderVoltage = (underVoltageCount == 0);
      info->capValEstimate = kickerBoard.getCapValEstimate();

      // 2024年版基板はドリブラーからのフィードバックがないため、フォトセンサのみでボール保持/検知を判定する
      // (2025年版基板では、DRIBBLE_RECVの受信時に保持/検知を更新する)
      if (!newDribblerDetected) {
            bool isBallDetected = (medianPhotoValue.calc(info->photoSensorValue) < PHOTOSENSOR_THRESHOLD);
            info->dribbleStatus.isDetectedBall = isBallDetected;
            info->dribbleStatus.isHoldBall = isBallDetected;
      }
}

void Robot::sendDribble(uint8_t power, bool forceSend) {
      static Timer timer;
      static uint8_t dribblePowerPrev = 255;  // 処理されるために255に初期化
      if (timer.read_ms() > 10000) timer.set_ms(10000);
      if (power == dribblePowerPrev && forceSend == false) {
            if (timer.read_ms() < 100)  // パワーが変わっていない場合は送信しない。 forceSendがtrueの場合は100msごとに送信する
                  return;
      }
      // 送り先は判別の結果で1つに決める (両方へ送るとCANの送信メールボックスが埋まり、モーターの指令を捨てることがある)。
      // 2025年版基板は、指令を受け付けられるようになる直前にDRIBBLE_RECVを送り始めるので、
      // 受信するまで2024年版基板へ送っていても、2025年版基板への指令が遅れることはない
      CANBus::CANData canData;
      if (newDribblerDetected) {
            canData = {
                .stdId = DRIBBLE_SEND,
                .data = {(uint8_t)((power != 0) ? 100 : 0), 0, 0, 0, 0, 0, 0, 0},  // 2025年版基板は0か100だけ
            };
      } else {
            canData = {
                .stdId = DRIBBLE,
                .data = {power, 0, 0, 0, 0, 0, 0, 0},
            };
      }
      can.send(canData);
      timer.reset();
      dribblePowerPrev = power;
}

void Robot::sendKicker(RobotInfo_t &info) {
      // --- doDirectの解除を最優先で判定する ---
      // キッカーボードは一度doDirectを武装すると、data[1]!=0xFFのコマンドを受けるまで
      // 解除されない。従来は「straight/chipのパワーが両方0のとき(else分岐)」でしか
      // resetDoDirectを送っていなかったため、Piが解除(doDirectKick=0)にしてもキック
      // パワーを送り続けるとelse分岐に入らず、ボードが武装したまま「戻らない」状態に
      // なっていた。ここでPiの指令(解除)とボードの自認(武装中)が食い違う場合は、
      // パワーの有無に関わらず確実に解除を送る。解除サイクルでは通常キックを打たない
      // よう早期returnする。
      if (!info.status.doDirectKick && info.kickerBoardDoDirectStatus.straight) {
            kickerBoard.resetDoDirect(STRAIGHT);
            return;
      }
      if (!info.status.doDirectChipKick && info.kickerBoardDoDirectStatus.chip) {
            kickerBoard.resetDoDirect(CHIP);
            return;
      }

      // キックの処理（ストレートを優先してキック）
      if (info.kicker.straight > 0) {
            kickerBoard.kick(STRAIGHT, info.kicker.straight, info.status.doDirectKick);
      } else if (info.kicker.chip > 0) {
            kickerBoard.kick(CHIP, info.kicker.chip, info.status.doDirectChipKick);
      }
}

void Robot::sendMotor(RobotInfo_t &info, uint8_t interval) {
      // MDの処理
      // モータードライバの送信速度決め
      int16_t __velX = meanVelX.calc((float)info.velX.vel);
      int16_t __velY = meanVelY.calc((float)info.velY.vel);
      int16_t __velAngler = meanVelAngler.calc((float)info.velAngler.vel);
      // ループ回数ではなく経過時間で送信間隔を決める
      // (回数だとループ周期のずれがそのまま送信周期に乗り、uint8_tの桁あふれでも間隔が乱れていた)
      static Timer timer;
      if (timer.read_ms() >= interval) {
            motorDriver.setVelocityFF(__velX, __velY, __velAngler);
            timer.reset();
      }
}

// FF速度ベクトルからロボットが止まっているかを判断する
void Robot::checkRobotRest(RobotInfo_t &info) {
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

void Robot::uiSendSerial(RobotInfo_t &info, uint16_t interval) {
      // UIにデータを送信する
      static const uint8_t HEADER = 0xFF;  // ヘッダ
      static const uint8_t dataSize = 21;  // 新UIに合わせて21バイトに変更
      static Timer timer;

      if (timer.read_ms() < interval) {
            return;
      }

      info.capaData.chargeState = info.isKickerChargeMode;
      info.capaData.chargeVal = info.capValEstimate;

      // DMA送信は1回のwriteで1パケットを渡す必要があるので、ヘッダも含めて組み立てる
      uint8_t packet[1 + dataSize] = {0};
      packet[0] = HEADER;
      uint8_t *buffer = &packet[1];
      buffer[0] = info.batteryVoltage;
      buffer[1] = info.capaData.data;
      buffer[2] = (uint8_t)info.buzzer;
      buffer[3] = info.dribbleStatus.isDetectedBall ? 1 : 0; // ボールセンサ

      // モーター速度 (float 4bytes x 4)
      float motorVel[4];
      for (int i = 0; i < 4; i++) {
          motorVel[i] = (float)info.mdStatus.motorAngularVelocity[i];
      }
      memcpy(&buffer[4], &motorVel[0], 4);
      memcpy(&buffer[8], &motorVel[1], 4);
      memcpy(&buffer[12], &motorVel[2], 4);
      memcpy(&buffer[16], &motorVel[3], 4);

      // モーターステータス (bitmask)
      // エラー検知がSTM側にない場合は、とりあえず全て正常(0x0F)とする
      buffer[20] = 0x0F;

      serial4.write(packet, sizeof(packet));
      // printf("send ui: bat=%d cap=%d buzzer=%d\n", buffer[0], info.capaData.chargeVal, buffer[2]);

      timer.reset();
}

void Robot::uiRecvSerial(RobotInfo_t &info) {
      // UIにデータを送信する
      static const uint8_t HEADER = 0xFF;   // ヘッダ
      static const uint8_t dataSize = 2;    // データのサイズ (modeStatusとtestCommandの2バイト)
      static bool headerReceived = false;   // ヘッダを受信したかどうか
      static uint8_t index = 0;             // 受信したデータのインデックスカウンター
      static uint8_t data[dataSize] = {0};  // 受信したデータ

      while (serial4.available()) {
            uint8_t receivedByte = serial4.read();
            if (!headerReceived) {
                  index = 0;
                  if (receivedByte == HEADER) headerReceived = true;
            } else {
                  data[index] = receivedByte;
                  index++;
                    if (index == dataSize) {
                          info.uiStatus.data = data[0];
                          info.testCommand = data[1];
                          headerReceived = false;
                          index = 0;
                    }
            }
      }
}

bool Robot::bnoConfigure() {
      // STM32だけリセットされた場合BNO055は前回の動作モードのまま残っているので、
      // 設定レジスタ(UNIT_SEL等)を書く前に必ずCONFIGモードへ戻す(切替に最大19ms)
      bno.setOperaitonMode(OPERATION_MODE_CONFIG);
      HAL_Delay(25);
      bno.setUnit(0, 0, 1, 0);  // acc:m/s^2, gyro:dps, euler:rad (getAcc/getGyro/getEulerのスケールに合わせる)
      bno.setPowerMode();
#if IMU_FUSION == IMU_FUSION_STM
      const uint8_t target = OPERATION_MODE_AMG;
#else
      const uint8_t target = OPERATION_MODE_IMUPLUS;
#endif
      // bno.setOperaitonMode(OPERATION_MODE_NDOF);  // 地磁気を使うとモーター等の磁気でyawが乱れる
      bno.setOperaitonMode(target);
      HAL_Delay(10);  // CONFIGからの切替に7ms
      uint8_t mode = 0xFF;
      return bno.readOperationMode(&mode) && (mode & 0x0F) == target;
}

void Robot::bnoGet(RobotInfo_t &info, uint16_t interval) {
      // I2Cの読み出しは割り込みで行い、ここでは「完了したデータの処理」と「次の読み出し開始」だけを行う
      // (ブロッキングで読むとBNO055のクロックストレッチで1回約2.4msメインループが止まっていた)
      if (!imuAvailable) {
            return;  // IMUを載せていない機体 (imuStateは0のまま)
      }
      static Timer timer;
      acc_t acc;
      gyro_t gyro;
      euler_t euler;
#ifdef BNO_FAULT_TEST
      // 試験用: 起動から8秒後に一度だけBNO055をCONFIGモードに戻し、データが0になる状態を再現する
      static volatile uint8_t faultInjected = 0;  // 0: まだ / 1以上: CONFIGになったと読み返せるまでに書いた回数 / 0xFF: 失敗
      if (faultInjected == 0 && HAL_GetTick() > 8000) {
            bno.recoverBus();
            faultInjected = 0xFF;
            for (uint8_t i = 1; i <= 20; i++) {
                  bno.setOperaitonMode(OPERATION_MODE_CONFIG);
                  HAL_Delay(25);
                  uint8_t mode = 0xFF;
                  if (bno.readOperationMode(&mode) && (mode & 0x0F) == OPERATION_MODE_CONFIG) {
                        faultInjected = i;
                        break;
                  }
            }
      }
#endif
      bool hasNewData = bno.fetchAsync(&acc, &gyro, &euler);
      if (timer.read_ms() >= interval && bno.startReadAsync()) {
            timer.reset();
      }
      if (!hasNewData) {
            return;
      }
      static uint32_t zeroRun = 0;  // 生の値が全て0のサンプルが続いた数
      if (bno.lastRawAllZero) {
            // BNO055がCONFIGモードのまま (起動時の設定が抜けた・BNO055だけ電源が瞬断した等)。
            // 0を正しい値として使わない (imu_validを立てない)。続いたら設定し直す
            static uint32_t lastReconfigTick = 0;
            imuZeroSampleCount++;
            if (++zeroRun >= IMU_ZERO_RECONFIG_COUNT && HAL_GetTick() - lastReconfigTick >= IMU_RECONFIG_MIN_INTERVAL_MS) {
                  zeroRun = 0;
                  lastReconfigTick = HAL_GetTick();
                  imuReconfigCount++;
                  bno.recoverBus();  // 割り込み読み出しの途中でも、状態を揃えてから書く
                  bnoConfigure();
            }
            return;
      }
      zeroRun = 0;
      imuLastUpdateTick = HAL_GetTick();  // Piへ送るIMUの値が新しいか (rasSendSerialのimu_valid)

#if IMU_FUSION == IMU_FUSION_STM
      // 呼び出し周期がブロッキング処理(printf等)で乱れても正しく積分できるよう、
      // 前回呼び出しからの実経過時間をMadgwickに渡す(固定dt厳禁)
      float dt = bnoDtTimer.read_us() / 1000000.0f;
      bnoDtTimer.reset();
      if (dt > IMU_MAX_DT_S) dt = IMU_MAX_DT_S;

      // --- 静止中のバイアス追従 ---
      // 起動時に測ったバイアスも温度等でゆっくり変わり、残りがyawのドリフトになる。止まっている間は
      // 測定値へ少しずつ寄せる。「FF速度指令が0(isRobotStopFF)」かつ「補正後の角速度が全軸小さい」状態が
      // 続いたときだけ行うので、押されて回っている等、実際に回転しているときは追従しない
      static uint32_t stillSinceTick = 0;
      const bool isCalm = fabsf(gyro.x - bnoGyroBias.x) < IMU_STILL_GYRO_DPS &&
                          fabsf(gyro.y - bnoGyroBias.y) < IMU_STILL_GYRO_DPS &&
                          fabsf(gyro.z - bnoGyroBias.z) < IMU_STILL_GYRO_DPS;
      if (!(info.isRobotStopFF && isCalm)) {
            stillSinceTick = HAL_GetTick();
      } else if (HAL_GetTick() - stillSinceTick >= IMU_STILL_TIME_MS) {
            bnoGyroBias.x += IMU_BIAS_TRACK_ALPHA * (gyro.x - bnoGyroBias.x);
            bnoGyroBias.y += IMU_BIAS_TRACK_ALPHA * (gyro.y - bnoGyroBias.y);
            bnoGyroBias.z += IMU_BIAS_TRACK_ALPHA * (gyro.z - bnoGyroBias.z);
            imuPersist.gyroBias = bnoGyroBias;
      }

      gyro.x -= bnoGyroBias.x;
      gyro.y -= bnoGyroBias.y;
      gyro.z -= bnoGyroBias.z;

      bnoFilter.setDeltaT(dt);
      bnoFilter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);  // gyroは[dps]のまま渡す

      info.imuState.acc = acc;
      info.imuState.gyro = gyro;
      info.imuState.euler.yaw = bnoFilter.getYawRadians();
      info.imuState.euler.roll = bnoFilter.getRollRadians();
      info.imuState.euler.pitch = bnoFilter.getPitchRadians();
#else
      info.imuState.acc = acc;
      info.imuState.gyro = gyro;
      info.imuState.euler = euler;
#endif
      // printf("Acc: %.6f %.6f\n", info.imuState.acc.x, info.imuState.acc.y);
}

// ジャイロの静止バイアスをIMU_CALIB_SAMPLE_COUNT回サンプリングして平均する(ブロッキング、機体を静止させること)
void Robot::bnoGyroCalibrate() {
      printf("IMU Calibration Start (keep the robot stationary)\n");
      HAL_Delay(100);  // AMGへ切り替えた直後は出力が揃っていないので待つ

      gyro_t sum = {0, 0, 0};
      for (uint16_t i = 0; i < IMU_CALIB_SAMPLE_COUNT; i++) {
            gyro_t gyro = bno.getGyro();
            sum.x += gyro.x;
            sum.y += gyro.y;
            sum.z += gyro.z;
            HAL_Delay(10);
      }
      bnoGyroBias.x = sum.x / IMU_CALIB_SAMPLE_COUNT;
      bnoGyroBias.y = sum.y / IMU_CALIB_SAMPLE_COUNT;
      bnoGyroBias.z = sum.z / IMU_CALIB_SAMPLE_COUNT;

      imuPersist.gyroBias = bnoGyroBias;
      imuPersist.magic = IMU_PERSIST_MAGIC;

      bnoFilter.setBeta(IMU_MADGWICK_BETA);
      bnoDtTimer.reset();
      printf("IMU Calibration Done  GyroBias[dps] X:%+.3f Y:%+.3f Z:%+.3f\n", bnoGyroBias.x, bnoGyroBias.y, bnoGyroBias.z);
}

void Robot::bnoCalibrate() {
      if (swImu.read() == false) {
            // // set flash
            printf("IMU calibrating\n");
            HAL_Delay(1000);
            bno.getOffset(&info.imuOffsets.acc, &info.imuOffsets.gyro);
            bno.setOffset(&info.imuOffsets.acc, &info.imuOffsets.gyro);
            printf("IMU calibrated\n");
            flash.writeFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            HAL_Delay(1000);
            flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            printf("ACC offset saved %.6f, %.6f, %.6f\n", info.imuOffsets.acc.x, info.imuOffsets.acc.y, info.imuOffsets.acc.z);
            printf("GYR offset saved %.6f, %.6f, %.6f\n", info.imuOffsets.gyro.x, info.imuOffsets.gyro.y, info.imuOffsets.gyro.z);
      } else {
            // load flash オフセット値をFlashから読み出す(初回起動時はimu resetスイッチを押して起動すること)
            flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            HAL_Delay(1000);
            printf("ACC offset loaded %.6f, %.6f, %.6f\n", info.imuOffsets.acc.x, info.imuOffsets.acc.y, info.imuOffsets.acc.z);
            printf("GYR offset loaded %.6f, %.6f, %.6f\n", info.imuOffsets.gyro.x, info.imuOffsets.gyro.y, info.imuOffsets.gyro.z);
            bno.setOffset(&info.imuOffsets.acc, &info.imuOffsets.gyro);
      }
}

void Robot::mpuget(RobotInfo_t &info) {
      if (mpu.isCalibrated() == true) {
            mpu.getAccGyro(&acc, &gyro, false);
            filter.updateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
            att.z = (float)MyMath::gapDegrees180(filter.getYaw(), info.frontDeg);
      }
}

void Robot::mpuSetup(RobotInfo_t &info) {
      if (swImu.read() == false) {
            // set flash
            printf("IMU calibrating\n");
            HAL_Delay(1000);
            mpu.calibrateAccGyro();
            mpu.getOffset(&info.mpu_imuOffsets.acc, &info.mpu_imuOffsets.gyro);
            flash.writeFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            HAL_Delay(1000);
            flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            printf("ACC offset saved %.6f, %.6f, %.6f\n", info.imuOffsets.acc.x, info.imuOffsets.acc.y, info.imuOffsets.acc.z);
            printf("GYR offset saved %.6f, %.6f, %.6f\n", info.imuOffsets.gyro.x, info.imuOffsets.gyro.y, info.imuOffsets.gyro.z);
      } else {
            // load flash オフセット値をFlashから読み出す(初回起動時はimu resetスイッチを押して起動すること)
            flash.loadFlash(FLASH_START_ADDRESS, (uint8_t *)&info.imuOffsets, sizeof(info.imuOffsets));
            mpu.setOffset(&info.mpu_imuOffsets.acc, &info.mpu_imuOffsets.gyro);
            printf("ACC offset loaded %.6f, %.6f, %.6f\n", info.imuOffsets.acc.x, info.imuOffsets.acc.y, info.imuOffsets.acc.z);
            printf("GYR offset loaded %.6f, %.6f, %.6f\n", info.imuOffsets.gyro.x, info.imuOffsets.gyro.y, info.imuOffsets.gyro.z);
            HAL_Delay(1000);
      }
      info.frontDeg = att.z;
}

void Robot::photoThresholdSet() {
      uint16_t thr = PHOTOSENSOR_THRESHOLD;
      CANBus::CANData data = {
          .stdId = PHOTOTHRESHOLD,
          .data = {
              (uint8_t)(thr & 0xFF),
              (uint8_t)((thr >> 8) & 0xFF),
          },
      };
      can.send(data);
}
