#pragma once

#ifdef DEBUG_SERIAL_USART3
#define UART_DMA_CHANNEL DMA1_Channel2
#endif

#ifdef DEBUG_SERIAL_USART2
#define UART_DMA_CHANNEL DMA1_Channel7
#endif

void SendTelemetry();

#ifdef CONTROL_APP_USART2
    void protocolByteReceived();
    void process_message(char *message);
    void process_message_overflow();
    #define PROTOCOL_STATE_IDLE 0
    #define PROTOCOL_STATE_WAIT_END 1
    #define PROTOCOL_STATE_SKIP_DOUBLE 2
    #define PROTOCOL_SOM '!'
    #define PROTOCOL_EOM '.'
#endif