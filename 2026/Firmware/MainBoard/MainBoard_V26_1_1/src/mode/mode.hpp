#ifndef __MODE_HPP_
#define __MODE_HPP_

#include "robot.hpp"
class Mode {
     public:
      Mode(Robot* robot) : robot(robot) {}

      virtual void Loop() = 0;

     protected:
      Robot* robot;
};

#endif  // __MODE_HPP_