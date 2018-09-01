#pragma once

#ifdef DEBUG_SERIAL_USART3
#define UART_DMA_CHANNEL DMA1_Channel2
#endif

#ifdef DEBUG_SERIAL_USART2
#define UART_DMA_CHANNEL DMA1_Channel7
#endif

void consoleScope();
void setScopeChannel(uint8_t ch, int16_t val);
void prepareAndSendTelemetry();