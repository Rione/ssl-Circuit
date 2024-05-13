#include <MotorDriver.hpp>

MotorDriver::MotorDriver(CANBus *canBus, BNO055 *bno) : _canBus(canBus), _bno(bno) {
}

void MotorDriver::init() {
}

void MotorDriver::setVelocityFF(int16_t velX, int16_t velY, float velAng) {

    /*================unit==============*/
    // velX, velY [mm/s]
    // velAng [rad/s]

    /*==============constans============*/
    // gear ratio 56：15
    static constexpr float gearRatio = (56 / 15);
    // wheel diameter 55mm
    const uint8_t wheelDiameter = 54;
    // robot wheel base diameter
    const uint8_t wheelBaseDiameter = 170;
    // robot spins → wheelRotate → motorRotate convert Ratio constant
    static constexpr float robotSpinToMotorRotateRatio = (wheelDiameter / wheelBaseDiameter) * gearRatio;
    // robot moves XY → wheelRotate → motoRotate convert constant
    static constexpr float velXYToMotorRotateRatio = gearRatio / (wheelDiameter * 3.141592653589);

    /*==============calculation============*/
    int16_t M0 = ((float)velX * MyMath::sinDeg(35) - (float)velY * MyMath::cosDeg(35)) * velXYToMotorRotateRatio + velAng * robotSpinToMotorRotateRatio;
    int16_t M1 = ((float)velX * MyMath::sinDeg(115) - (float)velY * MyMath::cosDeg(115)) * velXYToMotorRotateRatio + velAng * robotSpinToMotorRotateRatio;
    int16_t M2 = ((float)velX * MyMath::sinDeg(225) - (float)velY * MyMath::cosDeg(225)) * velXYToMotorRotateRatio + velAng * robotSpinToMotorRotateRatio;
    int16_t M3 = ((float)velX * MyMath::sinDeg(315) - (float)velY * MyMath::cosDeg(315)) * velXYToMotorRotateRatio + velAng * robotSpinToMotorRotateRatio;

    /*==============send data============*/

    CANBus::CANData data = {
        .stdId = 0x1AA,
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


