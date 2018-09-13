#include "stm32f1xx_hal.h"
#include "beeper.h"
#include "config.h"
#include "defines.h"
#include "control.h"
#include "comms.h"

extern uint8_t enable; // global variable for motor enable
extern uint8_t nunchuck_data[6];
uint8_t button1, button2;
int cmd1;  // normalized input values. -1000 to 1000
int cmd2;

int modeBeepDone = 0;
 
int8_t mode = 1, oldmode = 1;

int8_t startupModeSelect() {
  // enable motors
  enable = 1;

  // while power button pressed, mode can be changed
  while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      Nunchuck_Read();
      button1 = (uint8_t) (nunchuck_data[5]       & 1) ^ 1;
      button2 = (uint8_t)((nunchuck_data[5] >> 1) & 1) ^ 1;
      cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis
      cmd2 = CLAMP((nunchuck_data[1] - 128) * 8, -1000, 1000); // x - axis
      if(cmd1 < -500) {
        enable = 0;
        beepFreq(1,8);
        while (cmd1 < -500) {
          Nunchuck_Read();
          cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis
        }
      }
      if(cmd1 > 500) {
        mode=1;
        while (cmd1 > 500) {
          Nunchuck_Read();
          cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis
        }
      }

      if(cmd2 > 500) {
        mode=2;
        while (cmd2 > 500) {
          Nunchuck_Read();
          cmd2 = CLAMP((nunchuck_data[1] - 127) * 8, -1000, 1000); // x - axis
        }
      }
      if(cmd2 < -500) {
        mode=3;
        while (cmd2 < -500) {
          Nunchuck_Read();
          cmd2 = CLAMP((nunchuck_data[1] - 127) * 8, -1000, 1000); // x - axis
        }
      }

      mode = CLAMP(mode, 1, 3);
      if(mode != oldmode) {
        beep(mode);
        oldmode = mode;
        modeBeepDone = 1;
      }
      SendTelemetry(1);
      HAL_Delay(10);
  }

  if(!modeBeepDone) {
    HAL_Delay(100);
    beep(mode);
  }

  return mode;
}

