
#ifndef _SETUP_COM_H_
#define _SETUP_COM_H_

#define INLINE inline __attribute__((always_inline))

#define MODE_UNMATCH -1
#define IMU_CALIBURATION_2 2
#define IMU_CALIBURATION_3 3
#define IMU_CALIBURATION info.imuStatus == IMU_CALIBURATION_2 || info.imuStatus == IMU_CALIBURATION_3

#ifndef PI
#define PI 3.141592653589
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#endif