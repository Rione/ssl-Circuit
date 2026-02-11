#ifndef __UI_HPP_
#define __UI_HPP_

#include <stdint.h>

#include "BufferedSerial.hpp"

class UI {
     public:
      UI(BufferedSerial* serial);

      void Recv();
      void Send();

     private:
      BufferedSerial* serial_;
};

#endif  // __UI_HPP_