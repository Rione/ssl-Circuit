//basic_io_control

#include "Basic_IO_Control.hpp"

void Basic_IO_Control_Extension_Sensor::Ball_Sensor_Activate(){
  HAL_GPIO_WritePin(BS_LED_GPIO_Port, BS_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BS_OUT_GPIO_Port, BS_OUT_Pin, GPIO_PIN_SET);
}

void Basic_IO_Control_Extension_Sensor::Ball_Sensor_Inactivate(){
  HAL_GPIO_WritePin(BS_LED_GPIO_Port, BS_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BS_OUT_GPIO_Port, BS_OUT_Pin, GPIO_PIN_RESET);
}

void Basic_IO_Control_Extension_Sensor::ENC_Activate(){
  HAL_GPIO_WritePin(ENC_POWER_GPIO_Port, ENC_POWER_Pin, GPIO_PIN_SET);
}

void Basic_IO_Control_Extension_Sensor::ENC_Inactivate(){
  HAL_GPIO_WritePin(ENC_POWER_GPIO_Port, ENC_POWER_Pin, GPIO_PIN_RESET);
}

void Basic_IO_Control_LED::RED(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::YELLOW(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::BLUE(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::GREEN(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::CAN_LED(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::BS_LED(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(BS_LED_GPIO_Port, BS_LED_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(BS_LED_GPIO_Port, BS_LED_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::ALL_Control(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_LED::ALL_Control_EX_CAN(int status){
  if(status == HIGH){
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_SET);
  } else if(status == LOW){
    HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
  }
}

void Basic_IO_Control_Motor::Reverse(int speed){
  speed =  ((PWM_TIM3_FRQ_MAX - PWM_TIM3_FRQ_MIN + 1) / (100 - 1 + 1)) * speed;
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
}

void Basic_IO_Control_Motor::Forward(int speed){
  speed =  ((PWM_TIM3_FRQ_MAX - PWM_TIM3_FRQ_MIN + 1) / (100 - 1 + 1)) * speed;
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
}

void Basic_IO_Control_Motor::Brake(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MAX);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MAX);
}

void Basic_IO_Control_Motor::FET_DISABLE(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_TIM3_FRQ_MIN);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_TIM3_FRQ_MIN);
}

void Basic_IO_Control_Motor::ENABLE(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_SET);
}

void Basic_IO_Control_Motor::DISABLE(){
  HAL_GPIO_WritePin(MD_nSLEEP_GPIO_Port, MD_nSLEEP_Pin, GPIO_PIN_RESET);
}



