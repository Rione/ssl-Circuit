void set_led(uint8_t pin,uint8_t state){
  switch(pin){
    case red:
      if(state == HIGH){
        HAL_GPIO_WritePin(GPIOB,LED_RED_Pin,GPIO_PIN_SET);
      } else if(state == LOW){
        HAL_GPIO_WritePin(GPIOB,LED_RED_Pin,GPIO_PIN_RESET);
      }
    break;
    case blue:
      if(state == HIGH){
        HAL_GPIO_WritePin(GPIOB,LED_BLUE_Pin,GPIO_PIN_SET);
      } else if(state == LOW){
        HAL_GPIO_WritePin(GPIOB,LED_BLUE_Pin,GPIO_PIN_RESET);
      }
    break;
    default:
    break;
  }
}