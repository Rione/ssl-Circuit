#include "SparkFun_BMI270_Arduino_Library.h"
#include "arduino_compat/arduino_compat.h"
#include "testMode.hpp"
#include "MotorDriver.hpp"
#include "Robot.hpp"
#include "RobotSequence.hpp"
extern Robot robot;

//added for KF estimate
TestMode::TestMode(char letter, const char name[], Robot *robotPtr)
    : Mode(letter, name, robotPtr), estimator() {
  estimator.init(0.5f, 0.1f); // Initial parameters
  // If needed, set offsets from robot->info.imuOffsets
}

void TestMode::before()
{
    // Start serial
    Serial.begin(115200);
    timer.reset();
    dtTimer.reset();
    prevTime = 0.0f;
    Serial.println("TestMode::before() entered"); // ← 追加
    // Initialize the I2C library

    // Check if sensor is connected and initialize
    while(imu.beginI2C(i2cAddress, &hi2c1) != BMI2_OK)
    {
        // Wait a bit to see if connection is established
        Serial.println("beginI2C retry"); // ← 追加
        HAL_Delay(1000);
    }

    Serial.println("BMI270 connected!");

    int8_t err = BMI2_OK;

    // Set accelerometer config
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
    err = imu.setConfig(accelConfig);

    // Set gyroscope config
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_125;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(gyroConfig);

    // Check whether the config settings above were valid
    while(err != BMI2_OK)
    {
        if(err == BMI2_E_ACC_INVALID_CFG)
        {
            Serial.println("Accelerometer config not valid!");
        }
        else if(err == BMI2_E_GYRO_INVALID_CFG)
        {
            Serial.println("Gyroscope config not valid!");
        }
        else if(err == BMI2_E_ACC_GYR_INVALID_CFG)
        {
            Serial.println("Both configs not valid!");
        }
        else
        {
            Serial.print("Unknown error: ");
        }
        HAL_Delay(1000);
    }

    Serial.println("Configuration valid! Beginning measurements");
    HAL_Delay(5000);
}

void IMU(){
    // Get measurements from the sensor. This must be called before accessing
    // the sensor data, otherwise it will never update
    imu.getSensorData();
    imu_data_raw[0] = imu.data.accelX;
    imu_data_raw[1] = imu.data.accelY;
    imu_data_raw[2] = imu.data.gyroZ; //degree per second

    if(k < 1000){
        for (int i=0; i<3; i++){
            imu_data_total[i] += imu_data_raw[i];
        }
    }
    if (k == 999){
        for (int j=0; j<3; j++){
            imu_data_offset[j] = imu_data_total[j] / 1000.0;
        }
        printf("Calibration complete!\n");
        printf(" X: %.4f", imu_data_offset[0]);
        printf(" Y: %.4f", imu_data_offset[1]);
        printf(" Gyro: %.4f", imu_data_offset[2]);
        printf(" velocity:%d,fc:%f,gain:%f\n", target_velocity, fc, gain);
    }
    if (k == 1000){
        // Compute and optionally print acceleration-derived values
        for(int i=0; i<2; i++){
            imu_data_actual[i] = (imu_data_raw[i] - imu_data_offset[i])*9.80;
        }
            imu_data_actual[2] = (imu_data_raw[2] - imu_data_offset[2])*PI*(-1)/180.0;// convert degree to radian and minus offset error
        for(int i=0; i<3; i++){
            imu_data_filtered[i] = gain*imu_data_prev[i] + (1-gain)*imu_data_actual[i];
            //V[i] = V_prev[i] + (imu_data_filtered[i])* (dt/1000.0);
            imu_data_prev[i] = imu_data_filtered[i];
            //V_prev[i] = V[i];
        }

        robot.motorDriver.getVelocity(&velX, &velY, robot.info.mdStatus.motorAngularVelocity);
    }

    // NOTE: This function no longer blocks. Timing is managed by the caller
    // (TestMode::loop) using HAL_GetTick() so we don't use HAL_Delay here.

    if (k < 1000){
        k++;
    }
}
void TestMode::loop()
{
    static bool before_called = false;
    if (!before_called) {
        before();
        timer.reset();
        before_called = true;
    }

    if (k == 1000){
    robot->motorDriver.setVelocityFF(target_velocity,0, 0);// mm/s・mm/s・m rad/s,-500*PI, ロボット前向き:x軸positive,ロボット左:y軸positive
    }
    // 非ブロッキングでIMU取得を周期dt(ms)で行う
    int now = timer.read_ms();
    if ((int32_t)(now) >= dt) { // cast to signed to handle wrap safely, unit is "ms"
        IMU();
        if (k == 1000){
            //printf("Is it running?\n");
            // dt is in milliseconds in this loop; convert to seconds for the estimator
            estimator.update(imu_data_filtered[0], imu_data_filtered[1], 0.0f,
                             0.0f, 0.0f, imu_data_filtered[2],
                             0.001f * velX, 0.001f * velY, (float)dt / 1000.0f);
            printf("%f", imu_data_filtered[0]); printf("\t"); //accel_X
            printf("%f", imu_data_filtered[1]); printf("\t"); //accel_Y
            printf("%f", imu_data_filtered[2]); printf("\t"); //gyro_Z
            printf("%d", velX); printf("\t");
            printf("%f", estimator.getVelX()); printf("\t");
            printf("///");
            printf("%d", velY); printf("\t");
            printf("%f", estimator.getVelY()); printf("\t");
            printf("\n");
        }
        timer.reset();
    }
}


void TestMode::after()
{}