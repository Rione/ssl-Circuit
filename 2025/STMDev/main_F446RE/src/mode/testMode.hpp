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

/*Variables list*/
#define BMI2_I2C_PRIM_ADDR         0x68
// Create a new sensor object
BMI270 imu;
// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
int k = 0;
int dt = 10; //10ms
int target_velocity = 300; // [mm/s],X axis
float fc = 5.0;
float gain = 1/((dt*2*3.141592*fc*0.001)+1); //10Hz,15ms delay
int16_t velX = 0;
int16_t velY = 0;
int16_t gyro = 0;
float imu_data_raw[3] = {0.0, 0.0, 0.0};
float imu_data_total[3] = {0.0, 0.0, 0.0};
float imu_data_offset[3] = {0.0, 0.0, 0.0};
float imu_data_actual[3] = {0.0, 0.0, 0.0};
float imu_data_filtered[3] = {0.0, 0.0, 0.0};
float imu_data_prev[3] = {0.0, 0.0, 0.0};
float V[3] = {0.0, 0.0, 0.0};
float prev[3] = {0.0, 0.0, 0.0};
float V_prev[3] = {0.0, 0.0, 0.0};

#endif // TEST_MODE_HPP