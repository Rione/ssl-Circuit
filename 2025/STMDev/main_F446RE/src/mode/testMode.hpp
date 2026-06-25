#ifndef TEST_MODE_HPP
#define TEST_MODE_HPP
#include "Mode.hpp" // Adjusted relative path to where Mode.hpp resides
#include "Average.h"
#include "Median.h"
#include "RobotPoseEstimator.hpp"

class TestMode : public Mode {
  public:
    //TestMode(char letter, const char name[], Robot *robotPtr) : Mode(letter, name, robotPtr),
    TestMode(char letter, const char name[], Robot *robotPtr);
    /*estimator() {
      estimator.init(0.5f, 0.1f);
    }*/
    void before() override;
    void after() override;
    void loop() override;

  private:
    Timer timer;
    Timer dtTimer; // For calculating dt
    float prevTime;
    RobotPoseEstimator estimator;
};

#endif // TEST_MODE_HPP
