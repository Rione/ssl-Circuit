#include <MotorDriver.hpp>

MotorDriver::MotorDriver(CANBus *canBus) : _canBus(canBus) {
}

void MotorDriver::init() {
}

void MotorDriver::setVelocityFF(int16_t velX, int16_t velY, int16_t velAng) {
      // printf("%d, %d, %d\n", velX, velY, velAng);
      float _velX = (float)(velX);
      float _velY = (float)(velY);
      float _velAng = (float)(velAng) / 1000.0;
      /*================unit==============*/
      // velX, velY [mm/s]
      // velAng [m rad/s]
      /*==============constans============*/
      // gear ratio 56：15
      static constexpr float gearRatio = (56.0 / 15.0);
      // --------杉山機体のギア比 56:20
      // static constexpr float gearRatio = (56.0 / 20.0);
      // wheel diameter 55mm
      static constexpr float wheelDiameter = 54;
      // robot wheel base diameter
      static constexpr float wheelBaseDiameter = 170;
      // robot spins → wheelRotate → motorRotate convert Ratio constant
      static constexpr float robotSpinToMotorRotateRatio = (wheelBaseDiameter / wheelDiameter) * gearRatio;
      // robot moves XY → wheelRotate → motoRotate convert constant
      static constexpr float velXYToMotorRotateRatio = gearRatio / (wheelDiameter / 2);

      int16_t rotation = _velAng * -robotSpinToMotorRotateRatio;

      rotation = Constrain(rotation, -100, 100);

      /*==============calculation============*/
      int16_t M0 = (_velX * MyMath::sinDeg(55) - _velY * MyMath::cosDeg(55) * 1.05) * velXYToMotorRotateRatio + rotation;
      int16_t M1 = (_velX * MyMath::sinDeg(135) - _velY * MyMath::cosDeg(135)) * velXYToMotorRotateRatio + rotation;
      int16_t M2 = (_velX * MyMath::sinDeg(-135) - _velY * MyMath::cosDeg(-135)) * velXYToMotorRotateRatio + rotation;
      int16_t M3 = (_velX * MyMath::sinDeg(-55) - _velY * MyMath::cosDeg(-55) * 1.05) * velXYToMotorRotateRatio + rotation;

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