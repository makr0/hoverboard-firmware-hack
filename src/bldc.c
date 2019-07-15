
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "bldc.h"
#include "PID_controller.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW rtDW_Left;                    /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */

extern DW rtDW_Right;                   /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
boolean_T MotorControlOverrun = false;

// ###############################################################################

volatile ELECTRICAL_PARAMS electrical_measurements;

#define DO_MEASUREMENTS

volatile int pwml = 0;
int  pwml_limited = 0;
double pwml_regulated = 0;
volatile int pwmr = 0;
int pwmr_limited = 0;
double pwmr_regulated = 0;
int overCurrentL = 0;
int overCurrentR = 0;

extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;
extern int disablepoweroff;

uint32_t buzzerFreq    = 0;
uint32_t buzzerPattern = 0;
uint32_t buzzerTimer   = 0;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
// Careful - easy to use too many!
void DMA1_Channel1_IRQHandler() {

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
//#ifdef DO_MEASUREMENTS    
    electrical_measurements.batteryVoltage = batteryVoltage;
//#endif    
  }

  float dclAmps = ((float)ABS(adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP);
  float dcrAmps = ((float)ABS(adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP);

  electrical_measurements.motors[0].dcAmps = dclAmps;
  electrical_measurements.motors[1].dcAmps = dcrAmps;

#ifdef DO_MEASUREMENTS    
  electrical_measurements.motors[0].dcAmpsAvgAcc += ABS(adc_buffer.dcl - offsetdcl);
  electrical_measurements.motors[1].dcAmpsAvgAcc += ABS(adc_buffer.dcl - offsetdcl);

  if (buzzerTimer % 1000 == 500) { // to save CPU time
    electrical_measurements.motors[0].dcAmpsAvg = electrical_measurements.motors[0].dcAmpsAvgAcc*MOTOR_AMP_CONV_DC_AMP/1000;
    electrical_measurements.motors[1].dcAmpsAvg = electrical_measurements.motors[1].dcAmpsAvgAcc*MOTOR_AMP_CONV_DC_AMP/1000;
    electrical_measurements.motors[0].dcAmpsAvgAcc = 0;
    electrical_measurements.motors[1].dcAmpsAvgAcc = 0;
  }
#endif


  //disable PWM when on timeout or global disable
  if(timeout > TIMEOUT || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  }

  if(timeout > TIMEOUT || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }
  
  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }


  // soft limit current
  
  overCurrentL = (dclAmps - dynamicConfig.maxCurrent) * dynamicConfig.overcurrent.P;
  overCurrentR = (dcrAmps - dynamicConfig.maxCurrent) * dynamicConfig.overcurrent.P;

  pwml_limited = overCurrentL > 0 ? (pwml - overCurrentL * SIGN(pwml)) : pwml;
  pwmr_limited = overCurrentR > 0 ? (pwmr - overCurrentR * SIGN(pwmr)) : pwmr;
  

  // ############################### MOTOR CONTROL ###############################

  /* Check for overrun */
  if (MotorControlOverrun) {
    buzzerFreq = 80;
    buzzerPattern = 2;
    return;
  }
  MotorControlOverrun = true;
 
  int ul, vl, wl;
  int ur, vr, wr;
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    /* Set motor inputs here */
    rtU_Left.b_hallA   = hall_ul;
    rtU_Left.b_hallB   = hall_vl;
    rtU_Left.b_hallC   = hall_wl;
    rtU_Left.r_DC      = pwml_limited;
    
    /* Step the controller */
    BLDC_controller_step(rtM_Left);

    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;

    /* Apply commands */
    LEFT_TIM->LEFT_TIM_U    = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_V    = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_W    = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);
  // =================================================================
  

  // ========================= RIGHT MOTOR ===========================  
    // Get hall sensors values
    uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    /* Set motor inputs here */
    rtU_Right.b_hallA  = hall_ur;
    rtU_Right.b_hallB  = hall_vr;
    rtU_Right.b_hallC  = hall_wr;
    rtU_Right.r_DC     = pwmr_limited;

    /* Step the controller */
    BLDC_controller_step(rtM_Right);

    /* Get motor outputs here */
    ur            = rtY_Right.DC_phaA;
    vr            = rtY_Right.DC_phaB;
    wr            = rtY_Right.DC_phaC;
 // motSpeedRight = rtY_Right.n_mot;
 // motAngleRight = rtY_Right.a_elecAngle;

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V  = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W  = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  // =================================================================

  /* Indicate task complete */
  MotorControlOverrun = false;
 
 // ###############################################################################
}
