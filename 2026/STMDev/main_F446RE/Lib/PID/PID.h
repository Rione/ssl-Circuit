#ifndef PID_H
#define PID_H

#include "MyMath.hpp"
#include "Timer.hpp"

#ifdef __cplusplus

extern "C" {
class PID {
  private:
    /* data */
    float p, i, d; // PID gain
    float dt;      // interval of PID loop [s]
    float lastError;
    float error;

    float output, _output;
    float limit;
    float min, max;

    float pTerm;
    float iTerm, integral;
    float dTerm;

    Timer timer;

    void compute();

  public:
    PID(float _p, float _i, float _d, float _dt);
    void setLimit(float limit);
    void setGain(float _p, float _i, float _d);
    void reset();
    void appendError(float _error);
    float getPID();
};
}
#endif
#endif