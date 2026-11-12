#ifndef __BNO055__
#define __BNO055__

#include "DMAStream.h"
#include "MyMath.hpp"
#include "Timer.hpp"
#include "i2c.h"
#include "main.h"
#include "stdio.h"

#define BNOAddress (0x28 << 1)

// Register definitions
/* Page id register definition */
#define BNO055_PAGE_ID_ADDR 0x07
/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_REV_ID_ADDR 0x01
#define BNO055_MAG_REV_ID_ADDR 0x02
#define BNO055_GYRO_REV_ID_ADDR 0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR 0x06
/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D
/* Mag data register */
#define BNO055_MAG_DATA_X_LSB_ADDR 0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0x13
/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0x19
/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_EULER_H_MSB_ADDR 0x1B
#define BNO055_EULER_R_LSB_ADDR 0x1C
#define BNO055_EULER_R_MSB_ADDR 0x1D
#define BNO055_EULER_P_LSB_ADDR 0x1E
#define BNO055_EULER_P_MSB_ADDR 0x1F
/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27
/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D
/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR 0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR 0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR 0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR 0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR 0x33
/* Temperature data register */
#define BNO055_TEMP_ADDR 0x34
/* Status registers */
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_SELFTEST_RESULT_ADDR 0x36
#define BNO055_INTR_STAT_ADDR 0x37
#define BNO055_SYS_CLK_STAT_ADDR 0x38
#define BNO055_SYS_STAT_ADDR 0x39
#define BNO055_SYS_ERR_ADDR 0x3A
/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR 0x3B
#define BNO055_DATA_SELECT_ADDR 0x3C
/* Mode registers */
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_TEMP_SOURCE_ADDR 0x40
/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR 0x42
/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR 0x55
#define ACCEL_OFFSET_X_MSB_ADDR 0x56
#define ACCEL_OFFSET_Y_LSB_ADDR 0x57
#define ACCEL_OFFSET_Y_MSB_ADDR 0x58
#define ACCEL_OFFSET_Z_LSB_ADDR 0x59
#define ACCEL_OFFSET_Z_MSB_ADDR 0x5A
/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR 0x5B
#define MAG_OFFSET_X_MSB_ADDR 0x5C
#define MAG_OFFSET_Y_LSB_ADDR 0x5D
#define MAG_OFFSET_Y_MSB_ADDR 0x5E
#define MAG_OFFSET_Z_LSB_ADDR 0x5F
#define MAG_OFFSET_Z_MSB_ADDR 0x60
/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR 0x61
#define GYRO_OFFSET_X_MSB_ADDR 0x62
#define GYRO_OFFSET_Y_LSB_ADDR 0x63
#define GYRO_OFFSET_Y_MSB_ADDR 0x64
#define GYRO_OFFSET_Z_LSB_ADDR 0x65
#define GYRO_OFFSET_Z_MSB_ADDR 0x66
/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR 0x67
#define ACCEL_RADIUS_MSB_ADDR 0x68
#define MAG_RADIUS_LSB_ADDR 0x69
#define MAG_RADIUS_MSB_ADDR 0x6A

/* Page 1 registers */
#define BNO055_UNIQUE_ID_ADDR 0x50

// Definitions for unit selection
#define MPERSPERS 0x00
#define MILLIG 0x01
#define DEG_PER_SEC 0x00
#define RAD_PER_SEC 0x02
#define DEGREES 0x00
#define RADIANS 0x04
#define CENTIGRADE 0x00
#define FAHRENHEIT 0x10
#define WINDOWS 0x00
#define ANDROID 0x80

// Definitions for power mode
#define POWER_MODE_NORMAL 0x00
#define POWER_MODE_LOWPOWER 0x01
#define POWER_MODE_SUSPEND 0x02

// Definitions for operating mode
#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_ACCONLY 0x01
#define OPERATION_MODE_MAGONLY 0x02
#define OPERATION_MODE_GYRONLY 0x03
#define OPERATION_MODE_ACCMAG 0x04
#define OPERATION_MODE_ACCGYRO 0x05
#define OPERATION_MODE_MAGGYRO 0x06
#define OPERATION_MODE_AMG 0x07
#define OPERATION_MODE_IMUPLUS 0x08
#define OPERATION_MODE_COMPASS 0x09
#define OPERATION_MODE_M4G 0x0A
#define OPERATION_MODE_NDOF_FMC_OFF 0x0B
#define OPERATION_MODE_NDOF 0x0C

typedef struct {
      float yaw;
      float pitch;
      float roll;
} euler_t;

typedef struct {
      float x;
      float y;
      float z;
} acc_t;

typedef struct {
      float x;
      float y;
      float z;
} gyro_t;

typedef struct {
      float x;
      float y;
      float z;
} vel_t;

typedef struct {
      uint8_t sys;
      uint8_t gyr;
      uint8_t acc;
      uint8_t mag;
} calib_t;

class BNO055 {
     public:
      // scl/sda: I2Cバスがロックしたときに手動でクロックを送って復旧するためのピン
      BNO055(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *sclPort, uint16_t sclPin, GPIO_TypeDef *sdaPort, uint16_t sdaPin);
      // I2Cの途中でSTM32だけがリセットされると、BNO055がSDAをLowに保持したままになりバスが使えなくなる。
      // SCLを手動で最大9回送ってBNO055に転送を終わらせ、STOPを出してからI2Cを初期化し直す
      void recoverBus();
      bool check();
      void init();
      void setPowerMode();
      void setOperaitonMode(uint8_t mode);
      // OPR_MODEを読み返す。読めなければfalse (ブロッキング。起動時と設定し直すときだけ使う)
      bool readOperationMode(uint8_t *mode);
      void getCalibration();
      void setUnit(bool acc, bool gyro, bool euler, bool temp);
      void accConfig();

      euler_t getEuler();
      acc_t getAcc();
      gyro_t getGyro();

      vel_t getVel();

      void getOffset(acc_t *acc, gyro_t *gyro);
      void setOffset(acc_t *acc, gyro_t *gyro);

      float getAttitude();
      void setAttitudeZero();
      void setAngle(float rad);

      // --- 割り込みによる一括読み出し (メインループを止めない) ---
      // BNO055はクロックストレッチが長く、ブロッキングで読むと1回数msメインループが止まる。
      // startReadAsyncでACC/MAG/GYR/EULを1トランザクションで読み始め、完了後にfetchAsyncで取り出す。
      bool startReadAsync();  // 読み出し中ならfalse
      bool fetchAsync(acc_t *acc, gyro_t *gyro, euler_t *euler);  // 新しいデータがあればtrue
      void onAsyncComplete();  // HAL_I2C_MemRxCpltCallbackから呼ぶ
      void onAsyncError();     // HAL_I2C_ErrorCallbackから呼ぶ
      I2C_HandleTypeDef *getHi2c() { return _hi2c; }

      // 異常カウンタ
      uint32_t busRecoverCount = 0;           // 動作中にI2Cバスのロック・停止を検出して復旧した回数(起動時の復旧は数えない)
      volatile uint32_t i2cErrorCount = 0;    // 割り込み読み出しがエラーで終わった回数
      // busRecoverCountの内訳と、割り込み読み出しの結果 (I2Cの不調の切り分け用)
      uint32_t recoverTimeoutCount = 0;       // 読み出しがASYNC_TIMEOUT_MSのあいだ完了もエラーも来なかった
      uint32_t recoverBusyCount = 0;          // 次の読み出しを始めるときにバスがBUSYのままだった (STOPが出ていない等)
      uint32_t busySdaLowCount = 0;           //   そのときSDAがLow (BNO055が送り続けている: 最後のNACKをACKと読み違えた等)
      uint32_t busySclLowCount = 0;           //   そのときSCLがLow (どちらかがクロックを止めている)
      uint32_t busyRxDataCount = 0;           //   そのときSTM32が受信したバイトを持ったまま (SR1のRXNEかBTF。余分に1バイト受け取った)
      uint32_t busyMslCount = 0;              //   そのときSTM32がまだマスターのつもり (SR2のMSL。STOPを出せていない)
      uint32_t busyLinesHighCount = 0;        //   そのときSDAもSCLもHigh (線は空いているのにBUSYの印だけ残った)
      uint32_t recoverStillBusyCount = 0;     // 復旧の初期化のあともBUSYが残り、もう一度ソフトウェアリセットした
      uint32_t startFailCount = 0;            // HAL_I2C_Mem_Read_ITがHAL_OKを返さなかった
      volatile uint32_t asyncOkCount = 0;     // 割り込み読み出しが完了した回数
      volatile uint32_t errAfCount = 0;       // エラーの内訳: NACK (応答なし)
      volatile uint32_t errBerrCount = 0;     //               バスエラー (想定外のSTART/STOP。ノイズ・立ち上がりの遅れ)
      volatile uint32_t errArloCount = 0;     //               アービトレーション負け (SDAが思った値にならない)
      volatile uint32_t errOvrCount = 0;      //               オーバーラン
      // 直近のfetchAsyncで、加速度とジャイロの生の値が全て0だったか。止まっていても加速度には重力が
      // かかるので、全て0なのはBNO055がCONFIGモードのまま (データのレジスタが0) のとき
      bool lastRawAllZero = false;

      float frontRadians;

     private:
      static const uint8_t ASYNC_LEN = 24;               // 0x08(ACC_X_LSB)〜0x1F(EUL_P_MSB): ACC,MAG,GYR,EUL 各6byte
      static const uint32_t ASYNC_TIMEOUT_MS = 20;       // これ以上完了しなければI2Cを初期化し直す
      static const uint16_t RECOVER_SCL_WAIT_US = 2000;  // 復旧の前に、BNO055がSCLを放すのを待つ最長の時間
      uint8_t asyncBuf[ASYNC_LEN];
      volatile bool asyncBusy = false;
      volatile bool asyncReady = false;
      uint32_t asyncStartTick = 0;

      const uint16_t cnt_calib = 5000;

      float dt;  // サンプリング周期
      vel_t vel;
      acc_t prevAcc;
      Timer dt_timer;

      acc_t offsetAcc = {0, 0, 0};
      gyro_t offsetGyro = {0, 0, 0};
      I2C_HandleTypeDef *_hi2c;
      GPIO_TypeDef *_sclPort;
      uint16_t _sclPin;
      GPIO_TypeDef *_sdaPort;
      uint16_t _sdaPin;
};
#endif
