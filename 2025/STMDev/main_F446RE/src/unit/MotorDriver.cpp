#include <MotorDriver.hpp>

// 機体定数・三角関数の値は毎周期使うので起動時に一度だけfloatで求めておく
// (F446のFPUは単精度のみで、double演算はソフトウェア計算になり重い)
static const float SPIN_TO_MOTOR = (float)ROBOT_SPIN_TO_MOTOR_ROTATE_RATIO;
static const float XY_TO_MOTOR = (float)VELOCITY_XY_TO_MOTOR_ROTATE_RATIO;
static const float MOTOR_TO_WHEEL = (float)INV_GEAR_RATIO;
// 各ホイールの取り付け角(55, 135, -135, -55度)。M0,M3のcos成分は1.05倍の補正込み
static const float SIN_M0 = MyMath::sinDeg(55), COS_M0 = MyMath::cosDeg(55) * 1.05f;
static const float SIN_M1 = MyMath::sinDeg(135), COS_M1 = MyMath::cosDeg(135);
static const float SIN_M2 = MyMath::sinDeg(-135), COS_M2 = MyMath::cosDeg(-135);
static const float SIN_M3 = MyMath::sinDeg(-55), COS_M3 = MyMath::cosDeg(-55) * 1.05f;
// getVelocityの逆行列計算の分母
static const float VX_DENOM = XY_TO_MOTOR * (SIN_M0 * SIN_M0 + SIN_M1 * SIN_M1 + SIN_M2 * SIN_M2 + SIN_M3 * SIN_M3);
static const float VY_DENOM = XY_TO_MOTOR * (COS_M0 * COS_M0 + COS_M1 * COS_M1 + COS_M2 * COS_M2 + COS_M3 * COS_M3);

MotorDriver::MotorDriver(CANBus* canBus) : _canBus(canBus) {
}

void MotorDriver::init() {
}

void MotorDriver::setVelocityFF(int16_t velX, int16_t velY, int16_t velAng) {
      // printf("%d, %d, %d\n", velX, velY, velAng);
      float _velX = (float)(velX);
      float _velY = (float)(velY);
      float _velAng = (float)(velAng) / 1000.0f;
      /*================unit==============*/
      // velX, velY [mm/s]
      // velAng [m rad/s]
      //   /*==============constans============*/
      //   // gear ratio 56：15
      //   static constexpr float gearRatio = (56.0 / 15.0);
      //   // --------杉山機体のギア比 56:20
      //   // static constexpr float gearRatio = (56.0 / 20.0);
      //   // wheel diameter 54mm
      //   static constexpr float wheelDiameter = 54;
      //   // robot wheel base diameter
      //   static constexpr float wheelBaseDiameter = 170;
      //   // robot spins → wheelRotate → motorRotate convert Ratio constant
      //   static constexpr float robotSpinToMotorRotateRatio = (wheelBaseDiameter / wheelDiameter) * gearRatio;
      //   // robot moves XY → wheelRotate → motoRotate convert constant
      //   static constexpr float velXYToMotorRotateRatio = gearRatio / (wheelDiameter / 2);

      int16_t rotation = _velAng * -SPIN_TO_MOTOR;

      rotation = Constrain(rotation, -100, 100);

      /*==============calculation============*/
      int16_t M0 = (_velX * SIN_M0 - _velY * COS_M0) * XY_TO_MOTOR + rotation;
      int16_t M1 = (_velX * SIN_M1 - _velY * COS_M1) * XY_TO_MOTOR + rotation;
      int16_t M2 = (_velX * SIN_M2 - _velY * COS_M2) * XY_TO_MOTOR + rotation;
      int16_t M3 = (_velX * SIN_M3 - _velY * COS_M3) * XY_TO_MOTOR + rotation;

      /*==============send data============*/
      // printf("%f, %f, %f\n", _velX, _velY, _velAng);
      // printf("%f, %f, %f\n", gearRatio, robotSpinToMotorRotateRatio, velXYToMotorRotateRatio);
      // printf("%d, %d, %d, %d\n", M0, M1, M2, M3);
      setMotors(M0, M1, M2, M3);
}

void MotorDriver::setMotors(int16_t M0, int16_t M1, int16_t M2, int16_t M3) {
      CANBus::CANData data = {
          .stdId = MD_SEND,
          .data = {
              (uint8_t)(M0 & 0xFF),
              (uint8_t)((M0 >> 8) & 0xFF),
              (uint8_t)(M1 & 0xFF),
              (uint8_t)((M1 >> 8) & 0xFF),
              (uint8_t)(M2 & 0xFF),
              (uint8_t)((M2 >> 8) & 0xFF),
              (uint8_t)(M3 & 0xFF),
              (uint8_t)((M3 >> 8) & 0xFF),
          },
      };
      _canBus->send(data);
}

void MotorDriver::sendEmg() {
      CANBus::CANData data = {
          .stdId = SEND_EMG,
          .data = {0},
      };
      _canBus->send(data);
}

int16_t MotorDriver::motorToWheelScaled(int16_t motor_omega) {
      float wheel_omega = (float)motor_omega * MOTOR_TO_WHEEL;
      int16_t scaled = (int16_t)(wheel_omega * 100.0f);
      return Constrain(scaled, WHEEL_OMEGA_SCALE_MIN, WHEEL_OMEGA_SCALE_MAX);
}

void MotorDriver::getVelocity(int16_t* velX, int16_t* velY, int16_t* motorAngulerVelocity) {
      float M[4];
      for (int i = 0; i < 4; i++) {
            M[i] = motorAngulerVelocity[i];
      }

      // 回転成分を除去
      // Todo: もっと良い方法を考える
      float rot = (M[0] + M[1] + M[2] + M[3]) / 4.0f;
      for (int i = 0; i < 4; i++) {
            M[i] -= rot;
      }

      // 逆行列計算
      float vx = (M[0] * SIN_M0 + M[1] * SIN_M1 + M[2] * SIN_M2 + M[3] * SIN_M3) / VX_DENOM;
      float vy = -(M[0] * COS_M0 + M[1] * COS_M1 + M[2] * COS_M2 + M[3] * COS_M3) / VY_DENOM;

      *velX = vx;
      *velY = vy;
      // printf("velX:%d velY:%d\n", *velX, *velY);
}