//definishion_Control

#include "main.h"
#include "app.hpp"
#include "adc.h"

#ifndef __DC_HPP__
#define __DC_HPP__

//*** User Definishion BEGIN ***

#define MIYUKI_TYPE
//CODE MAP
/**
  1:FUBUKI old 
  9:SIRAYUKI old black
  11/5:HATSUYUKI new
  4:MIYUKI white
  5:SIKINAMI
**/

#ifdef __cplusplus
extern "C" {
  const bool Fnc_Administrator_Privilege = true;
}
#endif

#define Main_Power_Constant_Range 10
#define Motor_Base_Current_RANGE 10
#define Motor_Current_Differ_Tolerance 20
//*** User Definishion END ***

//*** System Definishion BEGIN ***
#define START       1
#define STOP        2
#define HOLD        3

#define HIGH    0
#define LOW     1 

#define MOTOR_CURRENT   0
#define BALL_SENSOR_VAL 1
#define ENC1_VAL        2
#define ENC2_VAL        3

#define PWM_TIM3_FRQ_MAX 800
#define PWM_TIM3_FRQ_MIN 1
//*** System Definishion END ***

//*** Parameter Set BEGIN ***
//1:FUBUKI TYPE
#ifdef FUBUKI_TYPE
  //Current_THRESHOLD
  #define Motor_Base_Current 80
  #define Main_Power_Constant 40
  //Sensor_THRESHOLD
  #define PHOTO_THRESHOLD 100
  #define MOTOR_CURRENT_THRESHOLD 13
#endif

//2:SIRAYUKI TYPE
#ifdef SIRAYUKI_TYPE
  //Current_THRESHOLD
  #define Motor_Base_Current 100
  #define Main_Power_Constant 45
  //Sensor_THRESHOLD
  #define PHOTO_THRESHOLD 100
  #define MOTOR_CURRENT_THRESHOLD 13
#endif

//3:HATSUYUKI TYPE
#ifdef HATSUYUKI_TYPE
  //Current_THRESHOLD
  #define Motor_Base_Current 130
  #define Main_Power_Constant 47
  //Sensor_THRESHOLD
  #define PHOTO_THRESHOLD 100
  #define MOTOR_CURRENT_THRESHOLD 30
#endif

//4:MIYUKI TYPE
#ifdef MIYUKI_TYPE
  //Current_THRESHOLD
  #define Motor_Base_Current 138
  #define Main_Power_Constant 60
  //Sensor_THRESHOLD
  #define PHOTO_THRESHOLD 100
  #define MOTOR_CURRENT_THRESHOLD 10
#endif

//test:SIKINAMI_TYPE
#ifdef SIKINAMI_TYPE
  //Current_THRESHOLD
  #define Motor_Base_Current 100
  #define Main_Power_Constant 50
  //Sensor_THRESHOLD
  #define PHOTO_THRESHOLD 100
  #define MOTOR_CURRENT_THRESHOLD 15
#endif


#endif


