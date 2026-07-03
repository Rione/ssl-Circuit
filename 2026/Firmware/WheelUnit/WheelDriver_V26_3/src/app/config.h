#ifndef CONFIG_H_
#define CONFIG_H_

#define TEMP_LIMIT 60             // 温度制限 [°C] (これを超えたら過熱保護で駆動停止)
#define TEMP_RESET_HYSTERESIS 10  // 過熱保護の復帰ヒステリシス [°C] (TEMP_LIMIT - この値まで冷えたら自動復帰)

#define SUPPLY_VOLTAGE_MIN_LIMIT 15
#define SUPPLY_VOLTAGE_MAX_LIMIT 30

#define SERIAL_SEND_INTERVAL_US 500  // シリアル送信間隔 [μs]

#define CONTROL_INTERVAL_US 100  // 制御周期 [μs]

// BLDCのパラメーター
#define POLE_PAIRS 8                // 極対数 (磁石の数/2)
#define MAX_ANGULAR_SPEED 150.0f    // 最大角速度 [rad/s]
#define MAX_ANGULAR_ACCEL 10000.0f  // 最大角加速度 [rad/s^2]
#define MAX_AMP_VOLT 5.0f           // 最大印加電圧 [V]

#endif  // CONFIG_H_