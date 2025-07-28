#include "app.hpp"

extern ADC_HandleTypeDef hadc1;

uint16_t adc_val_ch1[5] = {0, 0, 0, 0, 0}; // Array to hold ADC values

void Setup(void){
  //ADC initialization
  // int hal_restart_tim = -1;
  // bool hal_restart = false;

  // printf("\n*** Start ADC Initalization ***\n");

  // do{
  //   hal_restart = false;
  //   hal_restart_tim ++;

  //   printf("HAL ADC Start ---- ");
  //   HAL_ADC_Start(&hadc1);
  //   printf("Success!\n");
  //   HAL_Delay(500);

  //   printf("HAL ADC Start DMA ---- ");
  //   HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&adc_val_ch1,5);
  //   printf("Success!\n");
  //   HAL_Delay(500);

  //   for(int i = 0;i < 5;i++){
  //     int continue_num = 0;
  //     printf("Check ADC_Val_%d ---- ",i + 1);
  //     while(!(adc_val_ch1[i] >= 0)){
  //       HAL_Delay(10);
  //       continue_num ++;
  //       if(continue_num > 50){
  //         printf("FAIL!\n");
  //         if(hal_restart_tim >= 3){
  //           printf("\n[][][] Inisiate System Rest [][][]\n");
  //           HAL_NVIC_SystemReset();
  //         } else {
  //           printf("-- Restart HAL initialization\n");
  //           hal_restart = true; 
  //         }
  //       }
  //     }

  //     if(hal_restart == false){
  //       printf("Success!\n");
  //     }

  //     HAL_Delay(200);
  //   }
  // } while(hal_restart != false);

  // printf("ADC Is Operating Normally\n");
  // printf("*** ADC Initalization Acomplished ***\n");

  // HAL_Delay(500);
 
  
  HAL_Delay(500);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_val_ch1, 5);
  //HAL_ADC_Start(&hadc1);
  
  

}

void MainLoop(){
  
  // HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET);
  // //HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_CAN_GPIO_Port,LED_CAN_Pin,GPIO_PIN_SET);
  // HAL_Delay(1000);
  // HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET);
  // //HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(LED_CAN_GPIO_Port,LED_CAN_Pin,GPIO_PIN_RESET);
  // HAL_Delay(1000);

  // printf("Hello World!\n");
  printf("%u,%u,%u,%u,%u\n", adc_val_ch1[0], adc_val_ch1[1], adc_val_ch1[2], adc_val_ch1[3], adc_val_ch1[4]);
  HAL_Delay(100);
}

