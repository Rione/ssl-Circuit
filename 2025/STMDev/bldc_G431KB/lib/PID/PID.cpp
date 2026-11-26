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

void PID::compute(float step) {
    // please append error before compute
    float _error = error;
    integral += (abs(_output) > limit ? 0 : _error * step); // アンチワインドアップ

    pTerm = p * _error;                      // 比例
    iTerm = i * integral;                    // 積分
    dTerm = d * (_error - lastError) / step; // 微分
    lastError = _error;

    _output = (pTerm + iTerm + dTerm);
    output = Constrain(_output, min, max);
    // pc->printf("p:%f i:%f d:%f Output: %f _out:%f\n", pTerm, iTerm, dTerm, output, _output);
}

float PID::getPID() {
    // dt ごとに計算する。前はタイマーを戻さず、最初の dt の後は毎ループ計算して積分に毎回 dt を足していたので、
    // ループの周期が dt より短いと I が dt / 周期 倍 (旧機体の MD で約 37 倍) に効いていた
    // (Rione/ssl-RAVEN-Alula の lab/exp/2026-11-26-ident-md-pi-timer-oldgen-desk、F-261)。
    // 積分と微分には、前に計算してから実際に経った時間を使う。ただし drive() が呼ばれない間 (止めている間) の後の
    // 1 回目は経った時間が長いので、dt の 10 倍を超えたら dt を使う (積分が一度に跳ねないように)。
    float elapsed = timer.read();
    if (elapsed >= dt) {
        timer.reset();
        compute(elapsed > 10 * dt ? dt : elapsed);
    }
    return output;
}
