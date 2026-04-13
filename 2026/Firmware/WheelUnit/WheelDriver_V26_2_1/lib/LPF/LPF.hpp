#ifndef LPF_H
#define LPF_H

#ifdef __cplusplus
#include "Timer.hpp"
extern "C" {
class LPF {
  private:
    float timeConstant;
    float prevValue;
    Timer timer;

  public:
    LPF(float _timeConstant);
    float update(float _value);
};
}
#endif
#endif