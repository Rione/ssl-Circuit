#ifndef __MAIN_MODE_HPP_
#define __MAIN_MODE_HPP_

#include "mode.hpp"
#include "parammeter.hpp"

class MainMode : public Mode {
 public:
  MainMode(Robot* robot);

  void Loop() override;

 private:
};

#endif  // __MAIN_MODE_HPP_