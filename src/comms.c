#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "comms.h"
#include "bldc.h"
#include "hallinterrupts.h"
volatile ELECTRICAL_PARAMS electrical_measurements;

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

void SendTelemetry() {
    memset(uart_buf, 0, sizeof(uart_buf));
    if (telemetryTimer %20 == 0) {  // send voltage and Temperature only every 20th time this function is called
      sprintf(uart_buf, "*V%i*T%i*M%i*C%i*\r\n",
      (int)electrical_measurements.batteryVoltage*100, (int)electrical_measurements.board_temp_deg_c,mode,
      electrical_measurements.charging);
    } else {
      sprintf(uart_buf, "*s%i,%i*S%ld,%ld*A%i,%i*a%i*\r\n",
      speedR,speedL,
      HallData[0].HallSpeed_mm_per_s, HallData[1].HallSpeed_mm_per_s,
      (int)electrical_measurements.motors[0].dcAmps, (int)electrical_measurements.motors[1].dcAmps,
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