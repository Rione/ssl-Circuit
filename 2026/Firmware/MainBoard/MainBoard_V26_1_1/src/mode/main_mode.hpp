#ifndef __MAIN_MODE_HPP_
#define __MAIN_MODE_HPP_

#include "mode.hpp"

class MainMode : public Mode {
     public:
      MainMode(Robot* robot);
      void loop() override;
};

#endif  // __MAIN_MODE_HPP_