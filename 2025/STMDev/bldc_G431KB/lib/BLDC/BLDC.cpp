#include "BLDC.hpp"

BLDCMotor::BLDCMotor(PwmOut *_pwm, AS5048A *_encoder, uint8_t _polerQty, float _dt)
    : pwm(_pwm),
      encoder(_encoder),
      polePairQty(_polerQty),
      velocityPID(0.0001, 0, 0, _dt), // default gain
      velocityLPF(0.01),
      supplyVoltage(16),
      limitVoltage(5),
      elAngle(0),
      shAngle(0),
      shAnglePrev(0),
      shAngleZero(0),
      targetVelocity(0),
      limitVelocity(600),
      velocity(0),
      available(NOTSET),
      debug(false) {}

void BLDCMotor::init(bool setZeroPosInInit) {
    pwm->write(0, 0, 0);
    velocityPID.setLimit(8); // 2.3
    if (setZeroPosInInit) {
        setAbsoluteZero();
        printf("- set Zero Pos\n");
    } else {
        printf("skip set Zero Pos\n");
    }

    Diagnose(); // 故障診断
    available = true;
    printf("- BLDC init\n");
}

void BLDCMotor::setAbsoluteZero(float _shAngleZero) {
    if (_shAngleZero != NOTSET) {
        shAngleZero = _shAngleZero;
    } else {
        openLoopControl(1.5, HALF_PI);
        // wait(0.5);
        wait_ms(500);
        float _shAngle = 0;
        for (size_t i = 0; i < 10; i++) {
            _shAngle += updateEncoder();
        }
        shAngleZero = _shAngle / 10;

        if (debug) {
            if (shAngleZero == -1) {
                printf("- error: encoder not found");
            } else {
                printf("- shAngleZero:%f", shAngleZero);
            }
        }
        writePwm(0, 0, 0);
    }
}

void BLDCMotor::Diagnose() {
    available = NOTSET;
    if (debug) {
        printf("- Diagnose the Motor Driver..\n");
    }
    // if (abs(gapRadians(shAngle, Radians(deg))) > 0.1) {
    //     if (debug)
    //         pc->printf("- Error: Motor Driver is broken....\n\n\n\n\n");
    //     return;
    // }
    // 一旦諦め
    //  bool a, b, c, d = false;
    //  while (deg < 360 * polePairQty) {
    //      openLoopControl(1, Radians(deg));
    //      deg += 1;
    //      updateEncoder();
    //      if (abs(gapRadians(shAngle, HALF_PI)) < 0.1 && !a) {
    //          a = true;
    //          pc->printf("%d %d %d %d\n", a, b, c, d);
    //      };
    //      if (abs(gapRadians(shAngle, PI)) < 0.1 && !b) {
    //          b = true;
    //          pc->printf("%d %d %d %d\n", a, b, c, d);
    //      };
    //      if (abs(gapRadians(shAngle, 3 * HALF_PI)) < 0.1 && !c) {
    //          c = true;
    //          pc->printf("%d %d %d %d\n", a, b, c, d);
    //      };
    //      if (abs(gapRadians(shAngle, TWO_PI)) < 0.1 && !d) {
    //          d = true;
    //          pc->printf("%d %d %d %d\n", a, b, c, d);
    //      }
    //  }

    // available = a && b && c && d;
    available = 1;

    if (!available) {
        if (debug)
            printf("- Error: polerPairQty isn't correct or Motor Driver is broken....\n\n\n\n\n");
    } else {
        if (debug)
            printf("- This Motor Driber is available!!!\n\n\n\n\n");
    }
    writePwm(0, 0, 0);
    // wait(1);
    wait_us(1000000);
}

void BLDCMotor::setSupplyVoltage(float _supplyVoltage, float _limitVoltage) {
    supplyVoltage = _supplyVoltage;
    limitVoltage = _limitVoltage;
}

void BLDCMotor::setVelocityLimit(float _limit) {
    limitVelocity = _limit;
}

void BLDCMotor::setPIDGain(float _p, float _i, float _d) {
    velocityPID.setGain(_p, _i, _d);
    if (debug)
        printf("setGain P:%f I:%f D:%f\n", _p, _i, _d);
}

void BLDCMotor::setVelocity(float _velocity) {
    targetVelocity = Constrain(_velocity, -limitVelocity, limitVelocity);
    // if (debug)
    //     printf("setVelocity:%f\n", targetVelocity);
}

float BLDCMotor::getTargetVelocity() {
    return targetVelocity;
}

void BLDCMotor::writePwm(float pwmA, float pwmB, float pwmC) {
    if (available == false) return;
    pwm->write(pwmA, pwmB, pwmC);
    // if (debug)
    //     printf("%f %f %f\n", pwmA * 1000, pwmB * 1000, pwmC * 1000);
}

inline float BLDCMotor::updateEncoder() {
    float _angle = encoder->getAngleRad();
    shAngle = MyMath::gapRadians(shAngleZero, _angle);
    elAngle = MyMath::normalizeRadians(shAngle * polePairQty);
    return _angle;
}

float BLDCMotor::getShaftAngle() {
    return shAngle;
}

float BLDCMotor::getElectricAngle() {
    return elAngle;
}

float BLDCMotor::getAngularVelocity() {
    float dt = timer.read();
    timer.reset();
    float angleDiff = shAnglePrev - shAngle;
    if (angleDiff > PI) {
        angleDiff -= 2 * PI;
    } else if (angleDiff < -PI) {
        angleDiff += 2 * PI;
    }
    double angularVelocity = angleDiff / dt;
    if (abs(angleDiff) > PI && angularVelocity != 0) {
        angularVelocity += (angularVelocity > 0) ? -2 * PI / dt : 2 * PI / dt;
    }
    angularVelocity = velocityLPF.update(angularVelocity);
    shAnglePrev = shAngle;
    // if (debug)
    //     pc->printf("vel:%.4f sh:%f dt:%f\n", angularVelocity, shAngle, dt);
    return angularVelocity;
}

float BLDCMotor::getZeroPos() {
    return shAngleZero;
}

inline void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float _elAngle) {

#ifndef USE_TRAPEZOID_120

    static float T0, T1, T2;
    static float Ta, Tb, Tc;
    static float Ua, Ub, Uc;

    static uint8_t sector;

    static float Uout;
    // from SimpleFOC
    // a bit of optitmisation
    if (Ud) { // only if Ud and Uq set
        // _sqrt is an approx of sqrt (3-4% error)
        Uout = MyMath::sqrt(Ud * Ud + Uq * Uq) / limitVoltage;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        _elAngle = MyMath::normalizeRadians(_elAngle + MyMath::atan2(Uq, Ud));
    } else { // only Uq available - no need for atan2 and sqrt
        Uout = Uq / limitVoltage;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        // int8_t Dir = Uq > 0 ? 1 : -1;
        //魔法です。これ。
        float fwAngle = (targetVelocity * -0.005);
        if (fwAngle > 1.5) fwAngle = 1.5; // これは大体PI/2に合わせている。
        if (fwAngle < -1.5) fwAngle = -1.5;
        _elAngle = MyMath::normalizeRadians(_elAngle + HALF_PI) + fwAngle;
    }
    _elAngle = MyMath::normalizeRadians(_elAngle);

    Uq = abs(Uq);

    sector = (int)(_elAngle / PI_3) + 1;
    T1 = SQRT3 * MyMath::sin(sector * PI_3 - _elAngle) * Uout;
    T2 = SQRT3 * MyMath::sin(_elAngle - (sector - 1.0) * PI_3) * Uout;
    T0 = 0 - T1 - T2;
    // if (debug)
    //     pc->printf("elAngle:%f, sector:%d \n", _elAngle, sector);
    if (sector == 1) {
        Ta = T1 + T2 + T0 / 2;
        Tb = T2 + T0 / 2;
        Tc = T0 / 2;
    } else if (sector == 2) {
        Ta = T1 + T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T0 / 2;
    } else if (sector == 3) {
        Ta = T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T2 + T0 / 2;
    } else if (sector == 4) {
        Ta = T0 / 2;
        Tb = T1 + T0 / 2;
        Tc = T1 + T2 + T0 / 2;
    } else if (sector == 5) {
        Ta = T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T2 + T0 / 2;
    } else if (sector == 6) {
        Ta = T1 + T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T0 / 2;
    } else {
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

    Ua = Constrain(Ta * supplyVoltage, -limitVoltage, limitVoltage) + supplyVoltage / 2;
    Ub = Constrain(Tb * supplyVoltage, -limitVoltage, limitVoltage) + supplyVoltage / 2;
    Uc = Constrain(Tc * supplyVoltage, -limitVoltage, limitVoltage) + supplyVoltage / 2;

    // if (debug)
    //     pc->printf("%f %f %f\n", Ua, Ub, Uc);

    Ua /= supplyVoltage;
    Ub /= supplyVoltage;
    Uc /= supplyVoltage;

    writePwm(Ua, Ub, Uc);

#else
    static float Ua, Ub, Uc;
    static int trap_120_map[6][3] = {
        {0, 1, -1}, {-1, 1, 0}, {-1, 0, 1}, {0, -1, 1}, {1, -1, 0}, {1, 0, -1} // 120度ずつの位相差
    };
    int sector = 6 * (MyMath::normalizeRadians(_elAngle - HALF_PI) / TWO_PI);

    if (trap_120_map[sector][0] == 0) {
        Ua = 0.5;
        Ub = trap_120_map[sector][1] * Uq;
        Uc = trap_120_map[sector][2] * Uq;

    } else if (trap_120_map[sector][1] == 0) {
        Ua = trap_120_map[sector][0] * Uq;
        Ub = 0.5;
        Uc = trap_120_map[sector][2] * Uq;

    } else {
        Ua = trap_120_map[sector][0] * Uq;
        Ub = trap_120_map[sector][1] * Uq;
        Uc = 0.5;
    }
    writePwm(Ua, Ub, Uc);
#endif
}

inline void BLDCMotor::openLoopControl(float _Uq, float _elAngle) {
    // float a = _Uq * sin(_elAngle) + 0.5;
    // float b = _Uq * sin(_elAngle + PI_3) + 0.5;
    // float c = _Uq * sin(_elAngle - PI_3) + 0.5;
    // writePwm(a, b, c);
    setPhaseVoltage(_Uq, 0, _elAngle);
    wait_us(200);
}

void BLDCMotor::drive() {
    updateEncoder();
    velocity = getAngularVelocity();
    velocityPID.appendError(targetVelocity - velocity);
    float Uq = velocityPID.getPID();
    float Ud = 0;
    setPhaseVoltage(Uq, Ud, elAngle);
    // printf("V:%5.3f Uq:%5.3f \n", velocity, Uq);
}