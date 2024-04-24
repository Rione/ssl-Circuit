#include "app.h"
#include "DigitalInOut.hpp"
#include "PWMSingle.hpp"
#include "adc.h"
#include "usart.h"
#include <cstdio>
#include <cstring>
#include "PWMSingleN.hpp"

int data = 0;
DigitalOut debugled(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
DigitalOut canled(CAN_LED_GPIO_Port, CAN_LED_Pin);
DigitalOut charge(CHARGE_GPIO_Port, CHARGE_Pin);
PwmSingleOut straightkicker(&htim15, TIM_CHANNEL_2);
PwmSingleOut chipkicker(&htim3, TIM_CHANNEL_2);
PwmSingleOutN dribbler(&htim1, TIM_CHANNEL_2);

uint16_t adc_Value = 0;

void setup() {
    straightkicker.init();
    chipkicker.init();
    dribbler.init();
}

void main_app() {
    setup();
    printf("starttt\n\r");

    while (1) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        adc_Value = HAL_ADC_GetValue(&hadc1);
        printf("adc_Value = %d", adc_Value);
        HAL_UART_Transmit(&huart1, (uint8_t *)adc_Value, 1, 100);
        HAL_Delay(50);

        straightkicker.write(0.2);
        chipkicker.write(0.3);
        dribbler.write(0.08);
        charge = 0;
        debugled = !debugled;
        canled = !canled;
        HAL_Delay(500);
        data++;
    }
}