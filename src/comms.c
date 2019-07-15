#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "comms.h"
#include "BLDC_controller.h"           /* Motor Model's header file */
#include "bldc.h"
#include "hallinterrupts.h"
#include "beeper.h"
#include "control.h"
#include "operationmode.h"
#include <string.h>
#include <stdlib.h>
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;
extern P rtP;                           /* Block parameters (auto storage) */
// ###############################################################################
extern uint8_t enable; // global variable for motor enable

volatile ELECTRICAL_PARAMS electrical_measurements;

UART_HandleTypeDef huart2;

char uart_buf[255];
int16_t ch_buf[9];

#ifdef CONTROL_APP_USART2
  char protocolByte;
  
  typedef struct tag_PROTOCOL_STAT {
      char state;
      char nextState;
      int16_t count;
      char protocolBuffer[255];
  } PROTOCOL_STAT;
  PROTOCOL_STAT protocolStatus;
#endif

extern float board_temp_adc_filtered;
float board_temp_deg_c;
uint32_t telemetryTimer        = 0;
extern uint32_t timeout; // input timeout

extern int8_t operation_mode; // current operation operation_mode
extern int steer;  // steer Value
extern int speed;  // speed value
extern int speedR; // output speed: 0-1000
extern int speedL; // output speed: 0-1000
extern int speedL_regulated;
extern int speedR_regulated;
extern int pwml;
extern int pwmr;

extern ExtY rtY_Left;                   /* outputs from Motor Model left Motor */
extern ExtY rtY_Right;                  /* outputs from Motor Model right Motor*/
extern boolean_T MotorControlOverrun; // Motor Model was overrun. Considered an error

void SendTelemetry(int now) {
    memset(uart_buf, 0, sizeof(uart_buf));
    if (telemetryTimer %20 == 0 || now) {  // send voltage and Temperature only every 20th time this function is called
      sprintf(uart_buf,
      "*V%i*T%i*M%i*C%i"
      "*O%i*e%i*t%li"
      "*c%i*o%i"
      "*z%i*Z%i"
      "*y%i*Y%i"
      "*P%i*I%i*D%i\n",
      (int)(electrical_measurements.batteryVoltage*100), (int)electrical_measurements.board_temp_deg_c,operation_mode,electrical_measurements.charging,
      MotorControlOverrun?1:0,enable,timeout,
      dynamicConfig.maxCurrent, dynamicConfig.overcurrent.P,
      (int)(dynamicConfig.speedFilter * 1000),(int)(dynamicConfig.steerFilter * 1000),
      (int)(dynamicConfig.speedCoeff * 100), (int)(dynamicConfig.steerCoeff * 100),
      dynamicConfig.speedL.P,dynamicConfig.speedL.I,dynamicConfig.speedL.D);
    } else if (telemetryTimer %10 == 0 || now) {  // send voltage and Temperature only every 20th time this function is called
      sprintf(uart_buf,
      "*f%ld*g%ld",
      (int32_t)speedL_regulated,(int32_t)speedR_regulated);
    } else {
      sprintf(uart_buf, "*A%i**B%i"
                        "*a%i**b%i"
                        "*S%i"
                        "*G%i*\n",
      pwml,pwmr,                   // wanted motor speed
      rtY_Right.n_mot,rtY_Left.n_mot , // actual motor speed
      rtY_Right.n_mot+rtY_Left.n_mot , // summed motor speed for the big gauge      
      (int)(electrical_measurements.motors[0].dcAmps + electrical_measurements.motors[1].dcAmps));
    }

    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = strlen(uart_buf);
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
    telemetryTimer++;
}


/**
 * receive one character at a time
 * weird workaround needed: every character is received twice
 */
#ifdef CONTROL_APP_USART2
void protocolByteReceived() {
    switch(protocolStatus.state) {
      case PROTOCOL_STATE_IDLE:
        if (protocolByte == PROTOCOL_SOM){
            memset(protocolStatus.protocolBuffer, 0, sizeof(protocolStatus.protocolBuffer));
            protocolStatus.count = 0;
            protocolStatus.nextState = PROTOCOL_STATE_WAIT_END;
        }
        protocolStatus.state = PROTOCOL_STATE_SKIP_DOUBLE;
      break;
      case PROTOCOL_STATE_WAIT_END:
        if (protocolByte == PROTOCOL_EOM){
          protocolStatus.nextState = PROTOCOL_STATE_IDLE;
          process_message(protocolStatus.protocolBuffer);
        } else {
          protocolStatus.protocolBuffer[protocolStatus.count++] = protocolByte;
          if(protocolStatus.count+1 == sizeof(protocolStatus.protocolBuffer)) {
            protocolStatus.nextState = PROTOCOL_STATE_IDLE;
            process_message_overflow();
          }
        }
        protocolStatus.state = PROTOCOL_STATE_SKIP_DOUBLE;
      break;
      case PROTOCOL_STATE_SKIP_DOUBLE:
        protocolStatus.state = protocolStatus.nextState;
      break;
    }
}

/**
 * send message back, if rx buffer full
 */
void process_message_overflow() {
    memset(uart_buf, 0, sizeof(uart_buf));
    sprintf(uart_buf, "Message buffer overrun. try sending %c\r\n", PROTOCOL_EOM);
    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = strlen(uart_buf);
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
}

/**
 * interpret command and execute
 * possible commands (examples shown with StartChar=! and end char=. they are not included in message)
 * 
 * Setup/Tuning Commands, sent once per value change
 * 
 * PID commands. each component (P,I,D) can be omitted. Values are stored as integer
 * !ovrcP1234.!ovrcI42.!ovrcD23.    set overcurrentPID to P=1234 I=42 D=23
 * !speedP1234.!speedI42.!speedD23. set speedPID to P=1234 I=42 D=23
 * 
 * driving coefficients. are divided by 100 and stored as float values between 0 and 1
 * !speedC63.               set speed coefficient to 0.63
 * !steerC50.               set steer coefficient to 0.5
 * !speedF63.               set speed filter to 0.063 (factor1000!)
 * !steerF50.               set steer filter to 0.05 (factor1000!)
 * 
 * simple values. are stored as integers
 * !maxcN20.                set max overall current to 20A (10A per motor)
 * !cellN10.                set number of Cells to 10 (equals BAT_NUMBER_OF_CELLS on startup)
 * !modeN2.                 set operation_mode number to '2'
 *
 * Driving Commands, must be received at least 2 times per second, else motors are stopped (speeed=0, steer=0)
 * Values are stored as integer
 * !speedV100.              set speed value to 100. max=1000
 * !steerV100.              set steer value to 100. max=1000
 */

void process_message(char *message) {

  // MODE
  if(strStartsWith("modeN", message)) {
    operation_mode = atoi(message+strlen("modeN"));
    initializeConfigValues(operation_mode);
    beep(operation_mode);
  }

  // change motor control type
  if(strStartsWith("motorCT",message)) {
      int ct;
      ct = atoi(message+strlen("motorCT"));
      rtP.z_ctrlTypSel = ct;
      beep(ct);
  }
  // reset
  if(strStartsWith("reset",message)) {
    HAL_NVIC_SystemReset();
  }
  // reset
  if(strStartsWith("poweroff",message)) {
    poweroff();
  }
  // PID values
  if(strStartsWith("pidP", message)) {
    dynamicConfig.speedL.P = atoi(message+strlen("pidP"));
    dynamicConfig.speedR.P = dynamicConfig.speedL.P;
  }
  // PID values
  if(strStartsWith("pidI", message)) {
    dynamicConfig.speedL.I = atoi(message+strlen("pidI"));
    dynamicConfig.speedR.I = dynamicConfig.speedL.I;
  }
  // PID values
  if(strStartsWith("pidD", message)) {
    dynamicConfig.speedL.D = atoi(message+strlen("pidD"));
    dynamicConfig.speedR.D = dynamicConfig.speedL.D;
  }

  // max Current
  if(strStartsWith("maxcN", message)) dynamicConfig.maxCurrent = atoi(message+strlen("maxcN")) / 2;
  // overcurrent PID
  if(strStartsWith("ovrcP", message)) dynamicConfig.overcurrent.P = atoi(message+strlen("ovrcP"));
  // steering coeff
  if(strStartsWith("steerC", message)) dynamicConfig.steerCoeff = atoi(message+strlen("steerC")) / 100.0;
  // speed coeff
  if(strStartsWith("speedC", message)) dynamicConfig.speedCoeff = atoi(message+strlen("speedC")) / 100.0;

  // steering filter
  if(strStartsWith("steerF", message)) dynamicConfig.steerFilter = atoi(message+strlen("steerF")) / 1000.0;
  // speed filter
  if(strStartsWith("speedF", message)) dynamicConfig.speedFilter = atoi(message+strlen("speedF")) / 1000.0;

  // DRIVING CONTROL
  if(strStartsWith("steerV", message) || strStartsWith("speedV", message) || strStartsWith("stkX", message)) {
    remoteControl.lastCommandTick = HAL_GetTick();
    // steering
    if(strStartsWith("steerV", message)) remoteControl.steer = atoi(message+strlen("steerV"));
    // speed
    if(strStartsWith("speedV", message)) remoteControl.speed = atoi(message+strlen("speedV"));
    if(strStartsWith("stkX", message)) {
      char xstr[10], ystr[10];
      int xStart=0, yStart=0, i;
      for(i=0;i<strlen(message);i++) {
          if(message[i]=='X') xStart = i+1;
          if(message[i]=='Y') yStart = i+1;
      }
      for( i=0;i<yStart-xStart-1;i++) {
          xstr[i]=message[i+xStart];
      }
      xstr[i]=0;
      for( i=0;i<strlen(message)-yStart;i++) {
          ystr[i]=message[i+yStart];
      }
      ystr[i]=0;
      remoteControl.speed = - atoi(ystr);
      remoteControl.steer = atoi(xstr);
    }
  } 


  SendTelemetry(1);
}

int strStartsWith(char *pre, char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}
#endif