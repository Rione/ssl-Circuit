#ifndef LOCAL_CONTROLLER_HPP_
#define LOCAL_CONTROLLER_HPP_

#include "robot.hpp"

class LocalController {
 public:
  LocalController();

  void Stop(Robot& robot);  // ロボットを完全に停止させる

 private:
};

#endif  // LOCAL_CONTROLLER_HPP_
