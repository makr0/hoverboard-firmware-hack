#include "stm32f1xx_hal.h"
#include "config.h"
extern uint32_t buzzerFreq;
extern uint32_t buzzerPattern;


void beepFreq(uint8_t anzahl, uint8_t freq ) {  // blocking function, do not use in main loop!
    for(uint8_t i = 0; i < anzahl; i++) {
        buzzerFreq = freq;
        HAL_Delay(SOUND_DELAY_UP);
        buzzerFreq = 0;
        HAL_Delay(200);
    }
}

void beep(uint8_t anzahl ) {
    beepFreq(anzahl, 2);
}
