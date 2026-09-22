#include "PID.h"

PID::PID(float _p, float _i, float _d, float _dt) : p(_p), i(_i), d(_d), dt(_dt) {
    limit = 1000;
    setLimit(limit);
    lastError = 0;
    timer.reset();
}

void PID::setLimit(float _limit) {
    limit = abs(_limit);
    min = -limit;
    max = limit;
}
void PID::setGain(float _p, float _i, float _d) {
    p = _p;
    i = _i;
    d = _d;
}

void PID::appendError(float _error) {
    error = _error;
}

void PID::reset() {
    integral = 0;
    error = 0;
    lastError = 0;
}

void PID::compute() {
    // please append error before compute
    float _error = error;
    integral += (abs(_output) > limit ? 0 : _error * dt); // アンチワインドアップ

    pTerm = p * _error;                    // 比例
    iTerm = i * integral;                  // 積分
    dTerm = d * (_error - lastError) / dt; // 微分
    lastError = _error;

    _output = (pTerm + iTerm + dTerm);
    output = Constrain(_output, min, max);
    // pc->printf("p:%f i:%f d:%f Output: %f _out:%f\n", pTerm, iTerm, dTerm, output, _output);
}

float PID::getPID() {
    // if (dt != 0) compute();
    if (timer.read() > dt) compute();
    return output;
}
