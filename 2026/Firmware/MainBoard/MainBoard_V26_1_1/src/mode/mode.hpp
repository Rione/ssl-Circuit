#ifndef __MODE_HPP_
#define __MODE_HPP_

#include "../unit/robot.hpp"

class Mode {
     public:
      virtual ~Mode() = default;
      virtual void loop() = 0;
};

#endif  // __MODE_HPP_