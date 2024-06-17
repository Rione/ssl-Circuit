#include <Arduino.h>

typedef struct {
  union{
      struct{
          uint8_t mode : 6;
          bool emergencyStop : 1;    
          bool parity : 1;      
      };
      uint8_t data;
  }status;

  uint8_t modePrev = 0;

} UIModeSwitch_t;

UIModeSwitch_t modeSwitchData;

void ModeSend(uint8_t mode, bool emrg){ 
  static const uint8_t HEADER = 0xFF;
  
  modeSwitchData.status.mode = mode;
  modeSwitchData.status.emergencyStop = emrg;
  modeSwitchData.status.parity = true;

  // uint8_t data = 0b00000100;

  Serial1.write(HEADER);
  Serial1.write(modeSwitchData.status.data);
  Serial.write(modeSwitchData.status.data);
  // Serial.write(data);
}
