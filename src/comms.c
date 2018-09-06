#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "comms.h"

UART_HandleTypeDef huart2;

char uart_buf[120];
int16_t ch_buf[9];
//volatile char char_buf[300];
extern float board_temp_adc_filtered;
float board_temp_deg_c;
uint32_t telemetryTimer        = 0;
extern int8_t mode; // current operation mode
extern int steer;  // steer Value
extern int speed;  // speed value
extern int speedR; // output speed: 0-1000
extern int speedL; // output speed: 0-1000
extern float batteryVoltage; // battery voltage
extern float board_temp_deg_c;  // board temperature
extern float currentLeft;  // left motor current
extern float currentRight;  // right motor current

void setScopeChannel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void consoleScope() {
  #if defined DEBUG_SERIAL_SERVOTERM && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    uart_buf[0] = 0xff;
    uart_buf[1] = CLAMP(ch_buf[0]+127, 0, 255);
    uart_buf[2] = CLAMP(ch_buf[1]+127, 0, 255);
    uart_buf[3] = CLAMP(ch_buf[2]+127, 0, 255);
    uart_buf[4] = CLAMP(ch_buf[3]+127, 0, 255);
    uart_buf[5] = CLAMP(ch_buf[4]+127, 0, 255);
    uart_buf[6] = CLAMP(ch_buf[5]+127, 0, 255);
    uart_buf[7] = CLAMP(ch_buf[6]+127, 0, 255);
    uart_buf[8] = CLAMP(ch_buf[7]+127, 0, 255);
    uart_buf[9] = '\n';

    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = 10;
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
  #endif

  #if defined DEBUG_SERIAL_ASCII && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
  #ifndef DEBUG_SERIAL_APP
    memset(uart_buf, 0, sizeof(uart_buf));
    sprintf(uart_buf, "0:%i 1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i 8:%i\r\n",
     ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7],ch_buf[8]);
  #endif
  #ifdef DEBUG_SERIAL_APP
    memset(uart_buf, 0, sizeof(uart_buf));
    if (telemetryTimer %20 == 0) {  // send voltage and Temperature only every 20th time this function is called
      sprintf(uart_buf, "*V%i*T%i*M%i*\r\n",
      (uint16_t)batteryVoltage*100, (int16_t)board_temp_deg_c,mode);
    } else {
      sprintf(uart_buf, "*S%i,%i*A%i,%i*\r\n",
      ch_buf[0],ch_buf[1],ch_buf[2],ch_buf[3]);
    }
  #endif

    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = strlen(uart_buf);
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
  #endif
}
void prepareAndSendTelemetry() {
      // ####### CALC BOARD TEMPERATURE #######
      
      // ####### DEBUG SERIAL OUT #######
      #ifdef CONTROL_ADC
        setScopeChannel(0, (int)adc_buffer.l_tx2);  // 1: ADC1
        setScopeChannel(1, (int)adc_buffer.l_rx2);  // 2: ADC2
      #endif
      #ifdef DEBUG_NUNCHUCK
        for(int i=0; i<6;i++) {
          setScopeChannel(i, (int16_t)nunchuck_data[i]);
        }
      #endif

      #ifndef DEBUG_NUNCHUCK
        setScopeChannel(0, (int16_t)speedR); // output speed: 0-1000
        setScopeChannel(1, (int16_t)speedL); // output speed: 0-1000
        setScopeChannel(2, (int16_t)currentLeft);  // left motor current
        setScopeChannel(3, (int16_t)currentRight);  // right motor current
        #ifndef DEBUG_SERIAL_APP
          setScopeChannel(4, (int16_t)(batteryVoltage * 100.0f)); // battery voltage
          setScopeChannel(5, (int16_t)board_temp_deg_c);  // board temperature
        #endif

      #endif
      consoleScope();
      telemetryTimer++;
}