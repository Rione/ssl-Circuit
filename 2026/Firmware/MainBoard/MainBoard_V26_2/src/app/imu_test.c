#include "imu_test.h"

#include <stdio.h>

#include "lsm6dso32.h"
#include "spi.h"

static Lsm6dso32 imu;

void ImuTest_Run(void) {
  if (!Lsm6dso32_Init(&imu, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin,
                      LSM6DSO32_ODR_104HZ, LSM6DSO32_ACCEL_FS_4G,
                      LSM6DSO32_ODR_104HZ, LSM6DSO32_GYRO_FS_500DPS)) {
    printf("LSM6DSO32 Init Failed (WHO_AM_I mismatch)\n");
    while (1) {
      HAL_Delay(500);
    }
  }
  printf("LSM6DSO32 Init OK\n");

  while (1) {
    if (Lsm6dso32_DataReady(&imu)) {
      Lsm6dso32_Update(&imu);
      printf("Accel[g] X:%+.3f Y:%+.3f Z:%+.3f  Gyro[dps] X:%+.2f Y:%+.2f Z:%+.2f  Temp:%.1fC\n",
             imu.accelX, imu.accelY, imu.accelZ, imu.gyroX, imu.gyroY,
             imu.gyroZ, imu.temperature);
    }
    HAL_Delay(100);
  }
}
