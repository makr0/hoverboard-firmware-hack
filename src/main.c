/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "beeper.h"
#include "startup.h"
#include "comms.h"
#include "control.h"
#include "config.h"
#include "bldc.h"
#include "hallinterrupts.h"
//#include "hd44780.h"

#include <memory.h>
void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
//LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

int cmd1;  // normalized input values. -1000 to 1000
int cmd2;
int cmd3;


volatile Serialcommand command;

uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000
float steer_coefficient = DEFAULT_STEER_COEFFICIENT; // steering strength. is changed on button press

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000
extern int useBlockPWM;

float weakrFloat;  // for ramping up Turbo
float weaklFloat;  // for ramping up Turbo
float FILTER_var = FILTER; // variable speed input filter
float STEER_FILTER_var = STEER_FILTER; // variable steering input filter

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
int speedL = 0, speedR = 0;
float board_temp_deg_c;
int8_t mode;

uint32_t inactivity_timeout_counter;

extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

int milli_vel_error_sum = 0;


void poweroff() {
    buzzerPattern = 0;
    enable = 0;
    buzzerFreq = 6;
    HAL_Delay(SOUND_DELAY_DOWN);
    buzzerFreq = 0;
    HAL_Delay(SOUND_DELAY_DOWN);
    buzzerFreq = 8;
    HAL_Delay(SOUND_DELAY_DOWN);
    buzzerFreq = 0;

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);
    while(1) {}
}


int main(void) {
  HAL_Init();
  Interrupts_Config();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  Peripherals_Config();

  memset((void*)&electrical_measurements, 0, sizeof(electrical_measurements));

  // hold Power
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  #ifdef HALL_INTERRUPTS
    // enables interrupt reading of hall sensors for dead reconing wheel position.
    HallInterruptinit();
  #endif

  for (int i = 8; i >= 3; i--) {
    buzzerFreq = i;
    HAL_Delay(SOUND_DELAY_UP);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  int lastSpeedL = 0, lastSpeedR = 0;
  float board_temp_adc_filtered = (float)adc_buffer.temp;
  float batteryVoltage; // global variable for battery voltage

  mode = startupModeSelect();
  if(mode == 3) {
    STEER_FILTER_var = 0.5;
    FILTER_var = 0.2;
  }

  while(1) {
    HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms

    #ifdef CONTROL_NUNCHUCK
      Nunchuck_Read();
      cmd1 = CLAMP((nunchuck_data[0] - 127) * 8, -1000, 1000); // x - axis
      cmd2 = CLAMP((nunchuck_data[1] - 129) * 8, -1000, 1000); // y - axis

      button1 = (uint8_t) (nunchuck_data[5]       & 1) ^ 1;
      button2 = (uint8_t)((nunchuck_data[5] >> 1) & 1) ^ 1;
    #endif

    #ifdef CONTROL_PPM
      cmd1 = CLAMP((ppm_captured_value[0] - 500) * 2, -1000, 1000);
      cmd2 = CLAMP((ppm_captured_value[1] - 500) * 2, -1000, 1000);
      button1 = ppm_captured_value[5] > 500;
      float scale = ppm_captured_value[2] / 1000.0f;
    #endif

    #ifdef CONTROL_ADC
      // ADC values range: 0-4095, see ADC-calibration in config.h
      cmd1 = CLAMP(adc_buffer.l_tx2 - ADC1_MIN, 0, ADC1_MAX) / (ADC1_MAX / 1000.0f);  // ADC1
      cmd2 = CLAMP(adc_buffer.l_rx2 - ADC2_MIN, 0, ADC2_MAX) / (ADC2_MAX / 1000.0f);  // ADC2

      // use ADCs as button inputs:
      button1 = (uint8_t)(adc_buffer.l_tx2 > 2000);  // ADC1
      button2 = (uint8_t)(adc_buffer.l_rx2 > 2000);  // ADC2

      timeout = 0;
    #endif

    #ifdef CONTROL_SERIAL_USART2
      cmd1 = CLAMP((int16_t)command.steer, -1000, 1000);
      cmd2 = CLAMP((int16_t)command.speed, -1000, 1000);

      timeout = 0;
    #endif
    // ####### CALC BOARD TEMPERATURE #######
    board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
    board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;
    
    batteryVoltage = electrical_measurements.batteryVoltage; // copy battery voltage to local var
    electrical_measurements.board_temp_raw = adc_buffer.temp;
    electrical_measurements.board_temp_filtered = board_temp_adc_filtered;
    electrical_measurements.board_temp_deg_c = board_temp_deg_c;
    electrical_measurements.charging = !(CHARGER_PORT->IDR & CHARGER_PIN);

    useBlockPWM = !button2;


    // ####### LOW-PASS FILTER #######
    steer = steer * (1.0 - STEER_FILTER_var) + cmd1 * STEER_FILTER_var;
    speed = speed * (1.0 - FILTER_var) + cmd2 * FILTER_var;


    // ####### MIXER #######
    steer_coefficient = steer_coefficient * (1.0 - STEER_FILTER_var) + (button2 ? BUTTON_STEER_COEFFICIENT : DEFAULT_STEER_COEFFICIENT) * STEER_FILTER_var;
    speedR = CLAMP(speed * SPEED_COEFFICIENT -  steer * steer_coefficient, -1000, 1000);
    speedL = CLAMP(speed * SPEED_COEFFICIENT +  steer * steer_coefficient, -1000, 2000);

    // ####### MODE HANDLING ############
    if (mode == 1) {  // Mode 1, slow, max SPEED MODE1_MAX_SPEED
      speedR = CLAMP(speedR, -MODE1_MAX_SPEED, MODE1_MAX_SPEED);
      speedL = CLAMP(speedL, -MODE1_MAX_SPEED, MODE1_MAX_SPEED);
      weakl = 0;
      weakr = 0;
    }
    if (mode >= 2) {  // Mode 2, full speed, with turbo
      // ramp up turbo if speed over 800 and turbo button pressed
      if(HallData[0].HallSpeed > 800 && button1) {
        weaklFloat = weaklFloat * 0.95 + 450.0 * 0.05;
      } else {
      // ramp down turbo if slower
        weaklFloat = weaklFloat * 0.95;
      }
      if(HallData[1].HallSpeed > 800 && button1) {
        weakrFloat = weakrFloat * 0.95 + 450.0 * 0.05;
      } else {
        weakrFloat = weakrFloat * 0.95;
      }
      weakl = (int)weaklFloat;
      weakr = (int)weakrFloat;
    }


    // ####### SET OUTPUTS #######
    if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50) && timeout < TIMEOUT) {
    #ifdef INVERT_R_DIRECTION
      pwmr = speedR;
    #else
      pwmr = -speedR;
    #endif
    #ifdef INVERT_L_DIRECTION
      pwml = -speedL;
    #else
      pwml = speedL;
    #endif
    }

    lastSpeedL = speedL;
    lastSpeedR = speedR;


    if (inactivity_timeout_counter % 25 == 0) {
        SendTelemetry();
    }


    // ####### POWEROFF BY POWER-BUTTON #######
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}
      poweroff();
    }



    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && ABS(speed) < 20) || (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && ABS(speed) < 20)) {  // poweroff before mainboard burns OR low bat 3
      poweroff();
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      buzzerFreq = 4;
      buzzerPattern = 1;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
      buzzerFreq = 5;
      buzzerPattern = 42;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE) {  // low bat 2: fast beep
      buzzerFreq = 5;
      buzzerPattern = 6;
    } else if (BEEPS_BACKWARD && speed < -50) {  // backward beep
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else {  // do not beep
      buzzerFreq = 0;
      buzzerPattern = 0;
    }


    // ####### INACTIVITY TIMEOUT #######
    if (ABS(speedL) > 50 || ABS(speedR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter ++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      // the charger is plugged in, do not power off
      if (electrical_measurements.charging){
        inactivity_timeout_counter = 0;
      } else {
        poweroff();
      }
    }
  }
}

