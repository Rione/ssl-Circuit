#ifndef __MAIN_MODE_HPP_
#define __MAIN_MODE_HPP_

#include "../config/parammeter.hpp"
#include "mode.hpp"

class MainMode : public Mode {
     public:
      MainMode(Robot* robot);
      void loop() override;

     private:
      Timer timer;
};

#endif  // __MAIN_MODE_HPP_